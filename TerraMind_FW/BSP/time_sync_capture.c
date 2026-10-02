#include "time_sync_capture.h"

#include "tim.h"
#include <string.h>

/* Current CubeMX clock: APB1=60 MHz, TIMPRE=0, TIM2=120 MHz,
 * PSC=29 -> 4 MHz. The former 84 MHz / (20 + 1) setup is also valid.
 * A PPS period measures the real oscillator rate on each recording. */
#define TIMER_HZ TIME_SYNC_TIMER_HZ
#define MIN_PPS_PERIOD_TICKS (TIMER_HZ - TIMER_HZ / 8u)
#define MAX_PPS_PERIOD_TICKS (TIMER_HZ + TIMER_HZ / 8u)
#define DRDY_QUEUE_SIZE 256u
#define PPS_QUEUE_SIZE 16u
#define HISTORY_SIZE 32u
#define MAX_FRAME_DELAY_TICKS (TIMER_HZ / 100u) /* 10 ms */
#define GPS_WEEK_MS 604800000ull

static volatile uint32_t g_wraps;
static volatile uint8_t g_started;
static volatile TimeSyncEdge g_drdy_queue[DRDY_QUEUE_SIZE];
static volatile TimeSyncEdge g_pps_queue[PPS_QUEUE_SIZE];
static volatile TimeSyncEdge g_drdy_history[HISTORY_SIZE];
static volatile uint16_t g_drdy_head, g_drdy_tail;
static volatile uint8_t g_pps_head, g_pps_tail;
static volatile uint32_t g_drdy_sequence, g_pps_sequence;
static volatile uint32_t g_last_matched_drdy;
static volatile uint64_t g_last_pps_ticks;
static volatile TimeSyncEdge g_recent_pps;
static volatile TimeSyncStats g_stats;
static TimeSyncClockMap g_clock_map;
static int64_t g_last_candidate_offset;
static uint32_t g_last_candidate_sequence;

static uint32_t lock_interrupts(void)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}

static void unlock_interrupts(uint32_t mask)
{
    if (mask == 0u) __enable_irq();
}

uint8_t TimeSync_Init(void)
{
    RCC_ClkInitTypeDef clocks;
    uint32_t flash_latency;
    HAL_RCC_GetClockConfig(&clocks, &flash_latency);
    const uint64_t pclk = HAL_RCC_GetPCLK1Freq();
    uint64_t timer_clock;
    /* RM0433: TIMPRE=0 selects PCLK at /1, otherwise 2*PCLK.
     * TIMPRE=1 selects HCLK at /1,/2,/4, otherwise 4*PCLK. */
    if ((RCC->CFGR & RCC_CFGR_TIMPRE) == 0u)
        timer_clock = clocks.APB1CLKDivider == RCC_APB1_DIV1 ? pclk : 2u * pclk;
    else
        timer_clock = (clocks.APB1CLKDivider == RCC_APB1_DIV1 ||
                       clocks.APB1CLKDivider == RCC_APB1_DIV2 ||
                       clocks.APB1CLKDivider == RCC_APB1_DIV4) ?
                       HAL_RCC_GetHCLKFreq() : 4u * pclk;

    if (htim2.Instance != TIM2 || htim2.Init.Prescaler > 0xffffu ||
        htim2.Init.Period != UINT32_MAX ||
        htim2.Init.CounterMode != TIM_COUNTERMODE_UP ||
        TIM2->PSC != htim2.Init.Prescaler || TIM2->ARR != UINT32_MAX ||
        (TIM2->CR1 & (TIM_CR1_DIR | TIM_CR1_CMS)) != 0u ||
        timer_clock != (uint64_t)TIMER_HZ * (htim2.Init.Prescaler + 1u))
    {
        return 0u;
    }
    g_wraps = 0u;
    g_drdy_head = g_drdy_tail = 0u;
    g_pps_head = g_pps_tail = 0u;
    g_drdy_sequence = g_pps_sequence = g_last_matched_drdy = 0u;
    g_last_pps_ticks = 0u;
    memset((void *)&g_recent_pps, 0, sizeof(g_recent_pps));
    memset(&g_clock_map, 0, sizeof(g_clock_map));
    g_last_candidate_offset = 0;
    g_last_candidate_sequence = 0u;
    memset((void *)&g_stats, 0, sizeof(g_stats));
    g_stats.timer_hz_nominal = TIMER_HZ;
    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE | TIM_FLAG_CC1 |
                        TIM_FLAG_CC2 | TIM_FLAG_CC1OF | TIM_FLAG_CC2OF);
    if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK) return 0u;
    if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK)
    {
        HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
        return 0u;
    }
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    g_started = 1u;
    return 1u;
}

uint32_t TimeSync_Counter32(void)
{
    return g_started ? TIM2->CNT : 0u;
}

static uint64_t now_locked(void)
{
    const uint32_t counter = TIM2->CNT;
    const uint32_t pending_wrap = (TIM2->SR & TIM_SR_UIF) != 0u &&
                                  counter < 0x80000000u ? 1u : 0u;
    return ((uint64_t)(g_wraps + pending_wrap) << 32) | counter;
}

uint64_t TimeSync_NowTicks(void)
{
    if (!g_started) return 0u;
    const uint32_t mask = lock_interrupts();
    const uint64_t ticks = now_locked();
    unlock_interrupts(mask);
    return ticks;
}

uint64_t TimeSync_ExpandCounter(uint32_t counter)
{
    if (!g_started) return 0u;
    const uint64_t now = TimeSync_NowTicks();
    const uint32_t age = (uint32_t)((uint32_t)now - counter);
    return age <= 2u * TIMER_HZ ? now - age : 0u;
}

static void capture_drdy(uint32_t counter, uint32_t wraps)
{
    TimeSyncEdge edge;
    edge.sequence = ++g_drdy_sequence;
    edge.mcu_ms = HAL_GetTick();
    edge.ticks = ((uint64_t)wraps << 32) | counter;
    edge.period_ticks = 0u;
    g_drdy_history[edge.sequence & (HISTORY_SIZE - 1u)] = edge;
    const uint16_t next = (uint16_t)((g_drdy_head + 1u) & (DRDY_QUEUE_SIZE - 1u));
    if (next == g_drdy_tail) ++g_stats.drdy_queue_drops;
    else
    {
        g_drdy_queue[g_drdy_head] = edge;
        g_drdy_head = next;
    }
    g_stats.drdy_captures = edge.sequence;
}

static void capture_pps(uint32_t counter, uint32_t wraps)
{
    TimeSyncEdge edge;
    edge.sequence = ++g_pps_sequence;
    edge.mcu_ms = HAL_GetTick();
    edge.ticks = ((uint64_t)wraps << 32) | counter;
    const uint64_t interval = g_last_pps_ticks == 0u ? 0u : edge.ticks - g_last_pps_ticks;
    edge.period_ticks = interval <= UINT32_MAX ? (uint32_t)interval : 0u;
    g_last_pps_ticks = edge.ticks;
    g_recent_pps = edge;
    const uint8_t next = (uint8_t)((g_pps_head + 1u) & (PPS_QUEUE_SIZE - 1u));
    if (next == g_pps_tail) ++g_stats.pps_queue_drops;
    else
    {
        g_pps_queue[g_pps_head] = edge;
        g_pps_head = next;
    }
    g_stats.pps_captures = edge.sequence;
}

void TimeSync_OnTim2Irq(void)
{
    if (!g_started)
    {
        TIM2->SR = ~(TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF |
                     TIM_SR_CC1OF | TIM_SR_CC2OF);
        return;
    }
    const uint32_t status = TIM2->SR;
    const uint32_t enabled = TIM2->DIER;
    const uint32_t wraps = g_wraps;
    const uint32_t update = status & enabled & TIM_SR_UIF;
    const uint32_t capture1 = status & enabled & TIM_SR_CC1IF;
    const uint32_t capture2 = status & enabled & TIM_SR_CC2IF;
    const uint32_t ccr1 = capture1 ? TIM2->CCR1 : 0u;
    const uint32_t ccr2 = capture2 ? TIM2->CCR2 : 0u;
    const uint32_t clear = update | capture1 | capture2 |
                           (status & (TIM_SR_CC1OF | TIM_SR_CC2OF));
    if (clear != 0u) TIM2->SR = ~clear;
    if (status & TIM_SR_CC1OF) ++g_stats.drdy_overcaptures;
    if (status & TIM_SR_CC2OF) ++g_stats.pps_overcaptures;
    if (capture1)
        capture_drdy(ccr1, wraps + (update && ccr1 < 0x80000000u ? 1u : 0u));
    if (capture2)
        capture_pps(ccr2, wraps + (update && ccr2 < 0x80000000u ? 1u : 0u));
    if (update) g_wraps = wraps + 1u;
}

uint8_t TimeSync_MatchDrdy(uint32_t uart_end_counter, TimeSyncEdge *edge)
{
    if (!g_started || edge == NULL) return 0u;
    const uint32_t mask = lock_interrupts();
    uint32_t best_age = MAX_FRAME_DELAY_TICKS + 1u;
    uint32_t best_sequence = 0u;
    TimeSyncEdge best = {0};
    const uint32_t first = g_drdy_sequence > HISTORY_SIZE ?
                           g_drdy_sequence - HISTORY_SIZE + 1u : 1u;
    for (uint32_t seq = first; seq <= g_drdy_sequence; ++seq)
    {
        if (seq <= g_last_matched_drdy) continue;
        const volatile TimeSyncEdge *item =
            &g_drdy_history[seq & (HISTORY_SIZE - 1u)];
        if (item->sequence != seq) continue;
        const uint32_t age = uart_end_counter - (uint32_t)item->ticks;
        if (age <= MAX_FRAME_DELAY_TICKS && age < best_age)
        {
            best_age = age;
            best_sequence = seq;
            best.sequence = item->sequence;
            best.mcu_ms = item->mcu_ms;
            best.ticks = item->ticks;
            best.period_ticks = 0u;
        }
    }
    if (best_sequence != 0u)
    {
        g_last_matched_drdy = best_sequence;
        *edge = best;
    }
    else ++g_stats.imu_unmatched;
    unlock_interrupts(mask);
    return best_sequence != 0u;
}

void TimeSync_RecordUnmatched(void)
{
    const uint32_t mask = lock_interrupts();
    ++g_stats.imu_unmatched;
    unlock_interrupts(mask);
}

uint8_t TimeSync_PopDrdy(TimeSyncEdge *edge)
{
    if (edge == NULL || g_drdy_tail == g_drdy_head) return 0u;
    const volatile TimeSyncEdge *item = &g_drdy_queue[g_drdy_tail];
    edge->sequence = item->sequence;
    edge->mcu_ms = item->mcu_ms;
    edge->ticks = item->ticks;
    edge->period_ticks = 0u;
    g_drdy_tail = (uint16_t)((g_drdy_tail + 1u) & (DRDY_QUEUE_SIZE - 1u));
    return 1u;
}

uint8_t TimeSync_PopPps(TimeSyncEdge *edge)
{
    if (edge == NULL || g_pps_tail == g_pps_head) return 0u;
    const volatile TimeSyncEdge *item = &g_pps_queue[g_pps_tail];
    edge->sequence = item->sequence;
    edge->mcu_ms = item->mcu_ms;
    edge->ticks = item->ticks;
    edge->period_ticks = item->period_ticks;
    g_pps_tail = (uint8_t)((g_pps_tail + 1u) & (PPS_QUEUE_SIZE - 1u));
    return 1u;
}

void TimeSync_GetStats(TimeSyncStats *stats)
{
    if (stats == NULL) return;
    const uint32_t mask = lock_interrupts();
    memcpy(stats, (const void *)&g_stats, sizeof(*stats));
    unlock_interrupts(mask);
}

uint8_t TimeSync_BindGpsSecond(uint16_t week, uint32_t tow_ms,
                               uint64_t observed_ticks,
                               TimeSyncClockMap *mapping)
{
    if (!g_started || mapping == NULL || tow_ms >= GPS_WEEK_MS ||
        tow_ms % 1000u != 0u || observed_ticks == 0u) return 0u;
    const uint32_t mask = lock_interrupts();
    const TimeSyncEdge pps = {g_recent_pps.sequence, g_recent_pps.mcu_ms,
                              g_recent_pps.ticks, g_recent_pps.period_ticks};
    /* A 300 ms gate rejects a 500 ms pulse's trailing edge.  This remains
     * conditional until the UM982 PPS polarity and GPS time reference are checked. */
    if (pps.sequence == 0u || pps.period_ticks < MIN_PPS_PERIOD_TICKS ||
        pps.period_ticks > MAX_PPS_PERIOD_TICKS || observed_ticks < pps.ticks ||
        observed_ticks - pps.ticks > pps.period_ticks * 3u / 10u)
    {
        unlock_interrupts(mask);
        return 0u;
    }
    const int64_t gps_second = (int64_t)(((uint64_t)week * GPS_WEEK_MS + tow_ms) / 1000u);
    const int64_t offset = gps_second - pps.sequence;
    if (offset == g_last_candidate_offset &&
        pps.sequence == g_last_candidate_sequence + 1u)
    {
        if (g_clock_map.consecutive < 255u) ++g_clock_map.consecutive;
    }
    else g_clock_map.consecutive = 1u;
    g_last_candidate_offset = offset;
    g_last_candidate_sequence = pps.sequence;
    g_clock_map.pps_sequence = pps.sequence;
    g_clock_map.gps_week = week;
    g_clock_map.gps_tow_ms = tow_ms;
    g_clock_map.pps_ticks = pps.ticks;
    g_clock_map.period_ticks = pps.period_ticks;
    g_clock_map.serial_lag_ticks = (uint32_t)(observed_ticks - pps.ticks);
    g_clock_map.locked = g_clock_map.consecutive >= 3u ? 1u : 0u;
    *mapping = g_clock_map;
    unlock_interrupts(mask);
    return 1u;
}

uint8_t TimeSync_GetClockMap(TimeSyncClockMap *mapping)
{
    if (mapping == NULL || !g_started) return 0u;
    const uint64_t now = TimeSync_NowTicks();
    const uint32_t mask = lock_interrupts();
    *mapping = g_clock_map;
    if (mapping->locked &&
        (now < mapping->pps_ticks ||
         now - mapping->pps_ticks > 2u * TIMER_HZ + TIMER_HZ / 2u))
        mapping->locked = 0u;
    unlock_interrupts(mask);
    return mapping->locked;
}

uint8_t TimeSync_GpsToTicks(uint16_t week, uint32_t tow_ms,
                            uint64_t *ticks)
{
    if (ticks == NULL || tow_ms >= GPS_WEEK_MS) return 0u;
    TimeSyncClockMap map;
    if (!TimeSync_GetClockMap(&map)) return 0u;
    const int64_t target_ms = (int64_t)((uint64_t)week * GPS_WEEK_MS + tow_ms);
    const int64_t anchor_ms = (int64_t)((uint64_t)map.gps_week * GPS_WEEK_MS + map.gps_tow_ms);
    const int64_t difference_ms = target_ms - anchor_ms;
    if (difference_ms < -2000 || difference_ms > 2000) return 0u;
    const int64_t mapped = (int64_t)map.pps_ticks +
                           difference_ms * map.period_ticks / 1000;
    if (mapped < 0) return 0u;
    *ticks = (uint64_t)mapped;
    return 1u;
}

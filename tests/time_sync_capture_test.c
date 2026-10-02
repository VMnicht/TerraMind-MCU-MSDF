#include "../TerraMind_FW/BSP/time_sync_capture.h"
#include "tim.h"

#include <assert.h>
#include <string.h>

TIM_TypeDef fake_tim2;
TIM_HandleTypeDef htim2;
RCC_TypeDef fake_rcc;
static uint32_t g_pclk, g_hclk, g_apb_divider;

uint32_t HAL_RCC_GetPCLK1Freq(void) { return g_pclk; }
uint32_t HAL_RCC_GetHCLKFreq(void) { return g_hclk; }
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *clocks, uint32_t *flash_latency)
{
    clocks->APB1CLKDivider = g_apb_divider;
    *flash_latency = 4u;
}
uint32_t HAL_GetTick(void) { return 123u; }
int HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *handle, uint32_t channel)
{
    handle->Instance->DIER |= channel == TIM_CHANNEL_1 ? TIM_SR_CC1IF : TIM_SR_CC2IF;
    return HAL_OK;
}
int HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *handle, uint32_t channel)
{
    handle->Instance->DIER &= ~(channel == TIM_CHANNEL_1 ? TIM_SR_CC1IF : TIM_SR_CC2IF);
    return HAL_OK;
}
uint32_t __get_PRIMASK(void) { return 0u; }
void __disable_irq(void) {}
void __enable_irq(void) {}

static void configure(uint32_t hclk, uint32_t divider, uint32_t pclk,
                      uint32_t psc, uint32_t timpre)
{
    memset(&fake_tim2, 0, sizeof(fake_tim2));
    g_hclk = hclk;
    g_pclk = pclk;
    g_apb_divider = divider;
    fake_rcc.CFGR = timpre;
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = fake_tim2.PSC = psc;
    htim2.Init.Period = fake_tim2.ARR = UINT32_MAX;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
}

static void test_clock_configurations(void)
{
    static const struct
    {
        uint32_t hclk, divider, pclk, psc, timpre;
    } valid[] = {
        {168000000u, RCC_APB1_DIV4, 42000000u, 20u, 0u},
        {240000000u, RCC_APB1_DIV4, 60000000u, 29u, 0u},
        {64000000u, RCC_APB1_DIV1, 64000000u, 15u, 0u},
        {240000000u, RCC_APB1_DIV1, 240000000u, 59u, 0u},
        {240000000u, RCC_APB1_DIV2, 120000000u, 59u, 0u},
        {240000000u, RCC_APB1_DIV8, 30000000u, 14u, 0u},
        {256000000u, RCC_APB1_DIV16, 16000000u, 7u, 0u},
        {240000000u, RCC_APB1_DIV1, 240000000u, 59u, RCC_CFGR_TIMPRE},
        {240000000u, RCC_APB1_DIV2, 120000000u, 59u, RCC_CFGR_TIMPRE},
        {240000000u, RCC_APB1_DIV4, 60000000u, 59u, RCC_CFGR_TIMPRE},
        {240000000u, RCC_APB1_DIV8, 30000000u, 29u, RCC_CFGR_TIMPRE},
        {240000000u, RCC_APB1_DIV16, 15000000u, 14u, RCC_CFGR_TIMPRE}
    };
    for (unsigned i = 0; i < sizeof(valid) / sizeof(valid[0]); ++i)
    {
        configure(valid[i].hclk, valid[i].divider, valid[i].pclk,
                  valid[i].psc, valid[i].timpre);
        assert(TimeSync_Init());
        TimeSyncStats stats;
        TimeSync_GetStats(&stats);
        assert(stats.timer_hz_nominal == 4000000u);
    }

    configure(240000000u, RCC_APB1_DIV4, 60000000u, 20u, 0u);
    assert(!TimeSync_Init()); /* Old PSC on the new clock tree. */
    assert(fake_tim2.DIER == 0u);
    configure(240000000u, RCC_APB1_DIV4, 60000000u, 29u, RCC_CFGR_TIMPRE);
    assert(!TimeSync_Init()); /* TIMPRE changed without changing PSC. */
    configure(240000004u, RCC_APB1_DIV4, 60000001u, 29u, 0u);
    assert(!TimeSync_Init()); /* Reject a quotient that only rounds to 4 MHz. */
    configure(240000000u, RCC_APB1_DIV4, 60000000u, 29u, 0u);
    fake_tim2.PSC = 20u;
    assert(!TimeSync_Init());
    fake_tim2.PSC = 29u;
    htim2.Init.Period = 65535u;
    assert(!TimeSync_Init());
    htim2.Init.Period = UINT32_MAX;
    fake_tim2.ARR = 65535u;
    assert(!TimeSync_Init());
    fake_tim2.ARR = UINT32_MAX;
    htim2.Init.CounterMode = TIM_CR1_DIR;
    assert(!TimeSync_Init());
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    fake_tim2.CR1 = TIM_CR1_DIR;
    assert(!TimeSync_Init());
    fake_tim2.CR1 = TIM_CR1_CMS;
    assert(!TimeSync_Init());
    fake_tim2.CR1 = 0u;
    htim2.Init.Prescaler = fake_tim2.PSC = 65536u;
    assert(!TimeSync_Init());
    htim2.Instance = NULL;
    assert(!TimeSync_Init());
}

static void test_capture_and_gps_map(void)
{
    assert(TimeSync_Init());
    assert(fake_tim2.DIER == (TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF));
    fake_tim2.CNT = 1000000u;
    fake_tim2.CCR1 = 998000u;
    fake_tim2.SR = TIM_SR_CC1IF;
    TimeSync_OnTim2Irq();
    TimeSyncEdge edge;
    assert(TimeSync_PopDrdy(&edge) && edge.sequence == 1u && edge.ticks == 998000u);
    assert(TimeSync_MatchDrdy(1004000u, &edge) && edge.sequence == 1u);
    assert(!TimeSync_MatchDrdy(1004000u, &edge));

    fake_tim2.CNT = 2000000u;
    fake_tim2.CCR2 = 1999999u;
    fake_tim2.SR = TIM_SR_CC2IF;
    TimeSync_OnTim2Irq();
    assert(TimeSync_PopPps(&edge) && edge.sequence == 1u && edge.period_ticks == 0u);

    /* A pending update and a capture just after wrap share one IRQ. */
    fake_tim2.CNT = 30u;
    fake_tim2.CCR1 = 20u;
    fake_tim2.SR = TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC1OF;
    TimeSync_OnTim2Irq();
    assert(TimeSync_PopDrdy(&edge) && edge.sequence == 2u);
    assert(edge.ticks == ((uint64_t)1u << 32) + 20u);
    assert(TimeSync_NowTicks() == ((uint64_t)1u << 32) + 30u);
    assert(TimeSync_MatchDrdy(6020u, &edge) && edge.sequence == 2u);

    TimeSyncStats stats;
    TimeSync_GetStats(&stats);
    assert(stats.drdy_captures == 2u && stats.pps_captures == 1u);
    assert(stats.drdy_overcaptures == 1u && stats.imu_unmatched == 1u);

    fake_tim2.SR = fake_tim2.DIER = 0u;
    assert(TimeSync_Init());
    for (uint32_t i = 0u; i < 4u; ++i)
    {
        fake_tim2.CNT = 1000000u + i * 4000000u + 100000u;
        fake_tim2.CCR2 = 1000000u + i * 4000000u;
        fake_tim2.SR = TIM_SR_CC2IF;
        TimeSync_OnTim2Irq();
        TimeSyncClockMap map;
        if (i != 0u)
        {
            assert(TimeSync_BindGpsSecond(2438u, 31000000u + i * 1000u,
                                          fake_tim2.CNT, &map));
            assert(map.locked == (i == 3u));
        }
    }
    TimeSyncClockMap map;
    fake_tim2.SR = 0u; /* STM32 SR write-one leaves unrelated status bits unchanged. */
    assert(TimeSync_GetClockMap(&map) && map.consecutive == 3u);
    uint64_t mapped_ticks = 0u;
    assert(TimeSync_GpsToTicks(2438u, 31003500u, &mapped_ticks));
    assert(mapped_ticks == 15000000u);

    fake_tim2.CNT = (uint32_t)map.pps_ticks + 10000000u;
    assert(TimeSync_GetClockMap(&map)); /* Exactly 2.5 s is still valid. */
    ++fake_tim2.CNT;
    assert(!TimeSync_GetClockMap(&map));
    assert(!TimeSync_GpsToTicks(2438u, 31003500u, &mapped_ticks));
}

static void test_wrap_and_frame_window(void)
{
    configure(240000000u, RCC_APB1_DIV4, 60000000u, 29u, 0u);
    assert(TimeSync_Init());
    fake_tim2.CNT = 30u;
    fake_tim2.CCR1 = 20u;
    fake_tim2.CCR2 = UINT32_MAX - 10u;
    fake_tim2.SR = TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF;
    assert(TimeSync_NowTicks() == ((uint64_t)1u << 32) + 30u);
    TimeSync_OnTim2Irq();
    fake_tim2.SR = 0u;
    TimeSyncEdge edge;
    assert(TimeSync_PopPps(&edge) && edge.ticks == UINT32_MAX - 10u);
    assert(TimeSync_PopDrdy(&edge) && edge.ticks == ((uint64_t)1u << 32) + 20u);
    assert(TimeSync_ExpandCounter(UINT32_MAX - 10u) == UINT32_MAX - 10u);
    assert(TimeSync_MatchDrdy(40020u, &edge)); /* 10 ms inclusive. */
    fake_tim2.CCR1 = 50000u;
    fake_tim2.SR = TIM_SR_CC1IF;
    TimeSync_OnTim2Irq();
    assert(!TimeSync_MatchDrdy(90001u, &edge)); /* Just outside 10 ms. */
    fake_tim2.SR = 0u;
    fake_tim2.CNT = 9000000u;
    assert(TimeSync_ExpandCounter(1000000u) == ((uint64_t)1u << 32) + 1000000u);
    assert(TimeSync_ExpandCounter(999999u) == 0u);
}

static void test_pps_period_gate(void)
{
    const uint32_t periods[] = {3499999u, 3500000u, 4000000u, 4500000u, 4500001u, 5714286u};
    for (unsigned i = 0; i < sizeof(periods) / sizeof(periods[0]); ++i)
    {
        configure(240000000u, RCC_APB1_DIV4, 60000000u, 29u, 0u);
        assert(TimeSync_Init());
        fake_tim2.CCR2 = 1000000u;
        fake_tim2.SR = TIM_SR_CC2IF;
        TimeSync_OnTim2Irq();
        fake_tim2.CCR2 += periods[i];
        fake_tim2.SR = TIM_SR_CC2IF;
        TimeSync_OnTim2Irq();
        TimeSyncClockMap map;
        const uint8_t expected = periods[i] >= 3500000u && periods[i] <= 4500000u;
        assert(TimeSync_BindGpsSecond(2438u, 31001000u, fake_tim2.CCR2 + 100000u,
                                      &map) == expected);
        if (expected)
            assert(!TimeSync_BindGpsSecond(2438u, 31002000u,
                    fake_tim2.CCR2 + periods[i] * 3u / 10u + 1u, &map));
    }
}

int main(void)
{
    test_clock_configurations();
    configure(168000000u, RCC_APB1_DIV4, 42000000u, 20u, 0u);
    test_capture_and_gps_map();
    configure(240000000u, RCC_APB1_DIV4, 60000000u, 29u, 0u);
    test_capture_and_gps_map();
    configure(240000000u, RCC_APB1_DIV4, 60000000u, 59u, RCC_CFGR_TIMPRE);
    test_capture_and_gps_map();
    test_wrap_and_frame_window();
    test_pps_period_gate();
    return 0;
}

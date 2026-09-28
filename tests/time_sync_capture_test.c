#include "../TerraMind_FW/BSP/time_sync_capture.h"
#include "tim.h"

#include <assert.h>

TIM_TypeDef fake_tim2;
TIM_HandleTypeDef htim2 = {&fake_tim2, {20u}};
RCC_TypeDef fake_rcc;

uint32_t HAL_RCC_GetPCLK1Freq(void) { return 42000000u; }
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

int main(void)
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
    return 0;
}

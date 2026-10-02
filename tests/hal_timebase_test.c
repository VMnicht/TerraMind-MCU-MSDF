#include "stm32h7xx_hal.h"
#include <assert.h>

TIM_TypeDef fake_tim8;
RCC_TypeDef fake_rcc;
uint32_t uwTickPrio;
extern TIM_HandleTypeDef htim8;
static uint32_t g_hclk, g_pclk, g_divider;
static unsigned g_starts;

uint32_t HAL_RCC_GetHCLKFreq(void) { return g_hclk; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return g_pclk; }
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *clocks, uint32_t *latency)
{
    clocks->APB2CLKDivider = g_divider;
    *latency = 4u;
}
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority)
{
    assert(irq == TIM8_UP_TIM13_IRQn && priority == 5u && subpriority == 0u);
}
void HAL_NVIC_EnableIRQ(int irq) { assert(irq == TIM8_UP_TIM13_IRQn); }
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *handle)
{
    assert(handle->Instance == TIM8);
    fake_tim8.PSC = handle->Init.Prescaler;
    fake_tim8.ARR = handle->Init.Period;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *handle)
{
    ++g_starts;
    handle->Instance->DIER |= TIM_IT_UPDATE;
    return HAL_OK;
}

int main(void)
{
    static const struct
    {
        uint32_t hclk, divider, pclk, timpre, timer_clock;
    } valid[] = {
        {64000000u, RCC_APB2_DIV1, 64000000u, 0u, 64000000u}, /* HAL_Init before PLL. */
        {168000000u, RCC_APB2_DIV4, 42000000u, 0u, 84000000u},
        {240000000u, RCC_APB2_DIV4, 60000000u, 0u, 120000000u},
        {240000000u, RCC_APB2_DIV1, 240000000u, 0u, 240000000u},
        {240000000u, RCC_APB2_DIV2, 120000000u, 0u, 240000000u},
        {240000000u, RCC_APB2_DIV8, 30000000u, 0u, 60000000u},
        {240000000u, RCC_APB2_DIV16, 15000000u, 0u, 30000000u},
        {240000000u, RCC_APB2_DIV1, 240000000u, RCC_CFGR_TIMPRE, 240000000u},
        {240000000u, RCC_APB2_DIV2, 120000000u, RCC_CFGR_TIMPRE, 240000000u},
        {240000000u, RCC_APB2_DIV4, 60000000u, RCC_CFGR_TIMPRE, 240000000u},
        {240000000u, RCC_APB2_DIV8, 30000000u, RCC_CFGR_TIMPRE, 120000000u},
        {240000000u, RCC_APB2_DIV16, 15000000u, RCC_CFGR_TIMPRE, 60000000u}
    };
    for (unsigned i = 0; i < sizeof(valid) / sizeof(valid[0]); ++i)
    {
        g_hclk = valid[i].hclk;
        g_pclk = valid[i].pclk;
        g_divider = valid[i].divider;
        fake_rcc.CFGR = valid[i].timpre;
        const unsigned starts = g_starts;
        assert(HAL_InitTick(5u) == HAL_OK && g_starts == starts + 1u);
        assert(uwTickPrio == 5u && fake_tim8.ARR == 999u);
        assert(valid[i].timer_clock / (fake_tim8.PSC + 1u) / (fake_tim8.ARR + 1u) == 1000u);
        HAL_SuspendTick();
        assert((fake_tim8.DIER & TIM_IT_UPDATE) == 0u);
        HAL_ResumeTick();
        assert((fake_tim8.DIER & TIM_IT_UPDATE) != 0u);
    }
    const unsigned starts = g_starts;
    assert(HAL_InitTick(16u) == HAL_ERROR && g_starts == starts);
    fake_rcc.CFGR = 0u;
    g_divider = RCC_APB2_DIV1;
    g_pclk = 500000u;
    assert(HAL_InitTick(5u) == HAL_ERROR && g_starts == starts);
    g_pclk = 64000001u;
    assert(HAL_InitTick(5u) == HAL_ERROR && g_starts == starts);
    return 0;
}

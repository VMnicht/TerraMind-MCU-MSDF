#ifndef TEST_TIM_H
#define TEST_TIM_H

#include <stdint.h>

typedef struct
{
    uint32_t CNT;
    uint32_t SR;
    uint32_t DIER;
    uint32_t CCR1;
    uint32_t CCR2;
    uint32_t CR1;
    uint32_t PSC;
    uint32_t ARR;
} TIM_TypeDef;

typedef struct
{
    uint32_t Prescaler;
    uint32_t Period;
    uint32_t CounterMode;
    uint32_t ClockDivision;
} TIM_InitTypeDef;
typedef struct { TIM_TypeDef *Instance; TIM_InitTypeDef Init; } TIM_HandleTypeDef;
typedef struct { uint32_t CFGR; } RCC_TypeDef;
typedef struct { uint32_t APB1CLKDivider; uint32_t APB2CLKDivider; } RCC_ClkInitTypeDef;

extern TIM_TypeDef fake_tim2;
extern TIM_HandleTypeDef htim2;
extern RCC_TypeDef fake_rcc;
#define TIM2 (&fake_tim2)
#define RCC (&fake_rcc)

#define TIM_SR_UIF 1u
#define TIM_SR_CC1IF 2u
#define TIM_SR_CC2IF 4u
#define TIM_SR_CC1OF 512u
#define TIM_SR_CC2OF 1024u
#define TIM_FLAG_UPDATE TIM_SR_UIF
#define TIM_FLAG_CC1 TIM_SR_CC1IF
#define TIM_FLAG_CC2 TIM_SR_CC2IF
#define TIM_FLAG_CC1OF TIM_SR_CC1OF
#define TIM_FLAG_CC2OF TIM_SR_CC2OF
#define TIM_IT_UPDATE TIM_SR_UIF
#define TIM_CHANNEL_1 1u
#define TIM_CHANNEL_2 2u
#define RCC_CFGR_TIMPRE 0x8000u
#define RCC_APB1_DIV1 0u
#define RCC_APB1_DIV2 0x40u
#define RCC_APB1_DIV4 0x50u
#define RCC_APB1_DIV8 0x60u
#define RCC_APB1_DIV16 0x70u
#define TIM_COUNTERMODE_UP 0u
#define TIM_CR1_DIR 0x10u
#define TIM_CR1_CMS 0x60u
#define HAL_OK 0

#define __HAL_TIM_DISABLE(handle) ((handle)->Instance->CR1 = 0u)
#define __HAL_TIM_SET_COUNTER(handle, value) ((handle)->Instance->CNT = (value))
#define __HAL_TIM_CLEAR_FLAG(handle, flags) ((handle)->Instance->SR &= ~(flags))
#define __HAL_TIM_ENABLE_IT(handle, flags) ((handle)->Instance->DIER |= (flags))

uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *clocks, uint32_t *flash_latency);
uint32_t HAL_GetTick(void);
int HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *handle, uint32_t channel);
int HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *handle, uint32_t channel);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);

#endif

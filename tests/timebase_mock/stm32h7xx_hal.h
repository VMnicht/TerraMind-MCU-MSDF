#ifndef TEST_TIMEBASE_HAL_H
#define TEST_TIMEBASE_HAL_H

#include "../mock/tim.h"

typedef int HAL_StatusTypeDef;
extern TIM_TypeDef fake_tim8;
extern uint32_t uwTickPrio;
#define TIM8 (&fake_tim8)
#define HAL_ERROR 1
#define __NVIC_PRIO_BITS 4u
#define TIM8_UP_TIM13_IRQn 44
#define RCC_APB2_DIV1 RCC_APB1_DIV1
#define RCC_APB2_DIV2 RCC_APB1_DIV2
#define RCC_APB2_DIV4 RCC_APB1_DIV4
#define RCC_APB2_DIV8 RCC_APB1_DIV8
#define RCC_APB2_DIV16 RCC_APB1_DIV16
#define __HAL_RCC_TIM8_CLK_ENABLE() ((void)0)
#define __HAL_TIM_DISABLE_IT(handle, flags) ((handle)->Instance->DIER &= ~(flags))

uint32_t HAL_RCC_GetPCLK2Freq(void);
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority);
void HAL_NVIC_EnableIRQ(int irq);
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_InitTick(uint32_t priority);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);

#endif

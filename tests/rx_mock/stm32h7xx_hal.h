#pragma once
#include <stdint.h>
enum { HAL_UART_STATE_READY=32,HAL_UART_STATE_BUSY_RX=34 };
typedef struct { uint32_t RxState;uint8_t *buffer; } UART_HandleTypeDef;
typedef enum { HAL_OK=0,HAL_ERROR=1,HAL_BUSY=2 } HAL_StatusTypeDef;
#ifdef __cplusplus
extern "C" {
#endif
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
#ifdef __cplusplus
}
#endif

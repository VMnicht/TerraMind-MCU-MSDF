#ifndef TEST_STM32H7XX_HAL_H
#define TEST_STM32H7XX_HAL_H

#include <stdint.h>

typedef struct
{
    void *hdmatx;
} UART_HandleTypeDef;

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2
} HAL_StatusTypeDef;

#ifdef __cplusplus
extern "C" {
#endif
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *data,
                                        uint16_t length);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart);
uint32_t HAL_GetTick(void);
#ifdef __cplusplus
}
#endif

#endif

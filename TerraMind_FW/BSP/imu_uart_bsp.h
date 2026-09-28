#ifndef IMU_UART_BSP_H
#define IMU_UART_BSP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
}

class ImuUartBsp
{
public:
    static ImuUartBsp &instance();

    bool start(UART_HandleTypeDef *uart);
    void service();
    size_t read(uint8_t *destination, size_t capacity);
    size_t read_timed(uint8_t *destination, uint32_t *timer_counters,
                      size_t capacity);
    uint32_t overflow_count() const;
    uint32_t uart_error_count() const;
    uint32_t rearm_error_count() const;

    void on_rx_complete(UART_HandleTypeDef *uart);
    void on_uart_error(UART_HandleTypeDef *uart);

private:
    enum { RX_CAPACITY = 512u, RX_MASK = RX_CAPACITY - 1u };

    ImuUartBsp();
    bool arm_receive();

    UART_HandleTypeDef *uart_;
    uint8_t receive_byte_;
    uint8_t ring_[RX_CAPACITY];
    uint32_t timer_counters_[RX_CAPACITY];
    volatile uint16_t head_;
    volatile uint16_t tail_;
    volatile uint32_t overflow_count_;
    volatile uint32_t uart_error_count_;
    volatile uint32_t rearm_error_count_;
    volatile bool armed_;
};

#endif /* __cplusplus */
#endif

#ifndef MONITOR_UART6_DMA_H
#define MONITOR_UART6_DMA_H

#include <stddef.h>
#include <stdint.h>

extern "C" {
#include "stm32h7xx_hal.h"
}

typedef struct
{
    uint32_t records_queued;
    uint32_t records_dropped;
    uint32_t bytes_sent;
    uint32_t bytes_discarded;
    uint32_t dma_start_errors;
    uint32_t uart_errors;
    uint16_t queue_bytes;
    uint16_t queue_high_water;
    uint8_t dma_busy;
} MonitorTxStats;

class MonitorUart6Dma
{
public:
    static MonitorUart6Dma &instance();
    bool start(UART_HandleTypeDef *uart);
    void stop();
    bool enqueue(const char *record, size_t length);
    void service();
    MonitorTxStats stats() const;
    void on_tx_complete(UART_HandleTypeDef *uart);
    void on_uart_error(UART_HandleTypeDef *uart);

private:
    enum { QUEUE_CAPACITY = 8192u, QUEUE_MASK = QUEUE_CAPACITY - 1u };
    MonitorUart6Dma();
    void start_next_dma();
    UART_HandleTypeDef *uart_;
    alignas(32) uint8_t queue_[QUEUE_CAPACITY];
    volatile uint16_t head_;
    volatile uint16_t tail_;
    volatile uint16_t active_length_;
    volatile bool busy_;
    volatile bool failed_;
    volatile uint32_t records_queued_;
    volatile uint32_t records_dropped_;
    volatile uint32_t bytes_sent_;
    volatile uint32_t bytes_discarded_;
    volatile uint32_t dma_start_errors_;
    volatile uint32_t uart_errors_;
    volatile uint16_t queue_high_water_;
};

#endif

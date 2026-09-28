#include "monitor_uart6_dma.h"

#include <string.h>

MonitorUart6Dma::MonitorUart6Dma()
    : uart_(NULL), head_(0u), tail_(0u), active_length_(0u), busy_(false),
      failed_(false), records_queued_(0u), records_dropped_(0u),
      bytes_sent_(0u), bytes_discarded_(0u), dma_start_errors_(0u),
      uart_errors_(0u), queue_high_water_(0u)
{
}

MonitorUart6Dma &MonitorUart6Dma::instance()
{
    static MonitorUart6Dma transmitter;
    return transmitter;
}

bool MonitorUart6Dma::start(UART_HandleTypeDef *uart)
{
    if (uart == NULL || uart->hdmatx == NULL || uart_ != NULL) return false;
    uart_ = uart;
    return true;
}

bool MonitorUart6Dma::enqueue(const char *record, size_t length)
{
    if (record == NULL || length == 0u || length >= QUEUE_CAPACITY)
    {
        ++records_dropped_;
        return false;
    }
    const uint16_t free_bytes = static_cast<uint16_t>(
        (tail_ - head_ - 1u) & QUEUE_MASK);
    if (length > free_bytes)
    {
        ++records_dropped_;
        return false;
    }
    const uint16_t head = head_;
    const size_t until_end = static_cast<size_t>(QUEUE_CAPACITY - head);
    const size_t first = length < until_end ? length : until_end;
    memcpy(&queue_[head], record, first);
    if (length > first) memcpy(queue_, record + first, length - first);
    head_ = static_cast<uint16_t>((head + length) & QUEUE_MASK);
    ++records_queued_;
    const uint16_t queued = static_cast<uint16_t>((head_ - tail_) & QUEUE_MASK);
    if (queued > queue_high_water_) queue_high_water_ = queued;
    return true;
}

void MonitorUart6Dma::start_next_dma()
{
    if (uart_ == NULL || busy_ || failed_ || tail_ == head_) return;
    const uint16_t tail = tail_;
    const uint16_t length = static_cast<uint16_t>(
        head_ > tail ? head_ - tail : QUEUE_CAPACITY - tail);
    active_length_ = length;
    busy_ = true;
    if (HAL_UART_Transmit_DMA(uart_, &queue_[tail], length) != HAL_OK)
    {
        busy_ = false;
        active_length_ = 0u;
        ++dma_start_errors_;
    }
}

void MonitorUart6Dma::service()
{
    if (uart_ == NULL) return;
    if (failed_)
    {
        HAL_UART_AbortTransmit(uart_);
        // A failed DMA transfer may have emitted a partial record. Count all
        // outstanding bytes as discarded before resuming at a clean boundary.
        bytes_discarded_ += static_cast<uint16_t>((head_ - tail_) & QUEUE_MASK);
        tail_ = head_;
        active_length_ = 0u;
        busy_ = false;
        failed_ = false;
    }
    start_next_dma();
}

MonitorTxStats MonitorUart6Dma::stats() const
{
    MonitorTxStats out = {};
    out.records_queued = records_queued_;
    out.records_dropped = records_dropped_;
    out.bytes_sent = bytes_sent_;
    out.bytes_discarded = bytes_discarded_;
    out.dma_start_errors = dma_start_errors_;
    out.uart_errors = uart_errors_;
    out.queue_bytes = static_cast<uint16_t>((head_ - tail_) & QUEUE_MASK);
    out.queue_high_water = queue_high_water_;
    out.dma_busy = busy_ ? 1u : 0u;
    return out;
}

void MonitorUart6Dma::on_tx_complete(UART_HandleTypeDef *uart)
{
    if (uart != uart_ || !busy_) return;
    tail_ = static_cast<uint16_t>((tail_ + active_length_) & QUEUE_MASK);
    bytes_sent_ += active_length_;
    active_length_ = 0u;
    busy_ = false;
    start_next_dma(); // HAL has returned the UART to READY before this callback.
}

void MonitorUart6Dma::on_uart_error(UART_HandleTypeDef *uart)
{
    if (uart != uart_) return;
    ++uart_errors_;
    failed_ = true;
}

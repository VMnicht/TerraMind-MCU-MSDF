#include "imu_uart_bsp.h"
#include "time_sync_capture.h"

ImuUartBsp::ImuUartBsp()
    : uart_(NULL), receive_byte_(0u), head_(0u), tail_(0u),
      overflow_count_(0u), uart_error_count_(0u), rearm_error_count_(0u),
      armed_(false)
{
}

ImuUartBsp &ImuUartBsp::instance()
{
    static ImuUartBsp bus;
    return bus;
}

bool ImuUartBsp::arm_receive()
{
    if (uart_ == NULL)
    {
        return false;
    }
    if (HAL_UART_Receive_IT(uart_, &receive_byte_, 1u) != HAL_OK)
    {
        ++rearm_error_count_;
        armed_ = false;
        return false;
    }
    armed_ = true;
    return true;
}

bool ImuUartBsp::start(UART_HandleTypeDef *uart)
{
    if (uart == NULL || armed_)
    {
        return false;
    }
    uart_ = uart;
    head_ = 0u;
    tail_ = 0u;
    overflow_count_ = 0u;
    uart_error_count_ = 0u;
    rearm_error_count_ = 0u;
    return arm_receive();
}

void ImuUartBsp::service()
{
    if (!armed_ && uart_ != NULL && uart_->RxState == HAL_UART_STATE_READY)
    {
        arm_receive();
    }
}

size_t ImuUartBsp::read(uint8_t *destination, size_t capacity)
{
    return read_timed(destination, NULL, capacity);
}

size_t ImuUartBsp::read_timed(uint8_t *destination,
                             uint32_t *timer_counters, size_t capacity)
{
    if (destination == NULL)
    {
        return 0u;
    }
    size_t count = 0u;
    while (count < capacity && tail_ != head_)
    {
        destination[count] = ring_[tail_];
        if (timer_counters != NULL) timer_counters[count] = timer_counters_[tail_];
        ++count;
        tail_ = static_cast<uint16_t>((tail_ + 1u) & RX_MASK);
    }
    return count;
}

uint32_t ImuUartBsp::overflow_count() const
{
    return overflow_count_;
}

uint32_t ImuUartBsp::uart_error_count() const
{
    return uart_error_count_;
}

uint32_t ImuUartBsp::rearm_error_count() const
{
    return rearm_error_count_;
}

void ImuUartBsp::on_rx_complete(UART_HandleTypeDef *uart)
{
    if (uart != uart_)
    {
        return;
    }
    const uint16_t next = static_cast<uint16_t>((head_ + 1u) & RX_MASK);
    if (next == tail_)
    {
        ++overflow_count_;
    }
    else
    {
        ring_[head_] = receive_byte_;
        timer_counters_[head_] = TimeSync_Counter32();
        head_ = next;
    }
    armed_ = false;
    arm_receive();
}

void ImuUartBsp::on_uart_error(UART_HandleTypeDef *uart)
{
    if (uart != uart_)
    {
        return;
    }
    ++uart_error_count_;
    // HAL may retain an active receive on a recoverable error.
    armed_ = (uart_->RxState != HAL_UART_STATE_READY);
}

#include "um982_uart_bsp.h"
#include "time_sync_capture.h"

Um982UartBsp::Um982UartBsp()
    : uart_(NULL), receive_byte_(0u), head_(0u), tail_(0u),
      overflow_count_(0u), uart_error_count_(0u), rearm_error_count_(0u), armed_(false)
{
}

Um982UartBsp &Um982UartBsp::instance()
{
    static Um982UartBsp bus;
    return bus;
}

bool Um982UartBsp::arm_receive()
{
    if (uart_ == NULL) return false;
    // Prevent an already pending UART error from interrupting HAL's receive
    // setup; service() also trusts actual HAL state instead of a stale flag.
    const uint32_t mask=__get_PRIMASK();__disable_irq();
    const HAL_StatusTypeDef status=HAL_UART_Receive_IT(uart_, &receive_byte_, 1u);
    armed_=uart_->RxState!=HAL_UART_STATE_READY;
    __set_PRIMASK(mask);
    if (status != HAL_OK)
    {
        ++rearm_error_count_;
        return false;
    }
    return true;
}

bool Um982UartBsp::start(UART_HandleTypeDef *uart)
{
    if (uart == NULL || armed_) return false;
    uart_ = uart;
    head_ = 0u;
    tail_ = 0u;
    overflow_count_ = 0u;
    uart_error_count_ = 0u;
    rearm_error_count_ = 0u;
    return arm_receive();
}

void Um982UartBsp::service()
{
    if (uart_ != NULL && uart_->RxState == HAL_UART_STATE_READY)
    {
        arm_receive();
    }
}

size_t Um982UartBsp::read(uint8_t *destination, size_t capacity)
{
    return read_timed(destination, NULL, capacity);
}

size_t Um982UartBsp::read_timed(uint8_t *destination,
                               uint32_t *timer_counters, size_t capacity)
{
    if (destination == NULL) return 0u;
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

uint32_t Um982UartBsp::overflow_count() const { return overflow_count_; }
uint32_t Um982UartBsp::uart_error_count() const { return uart_error_count_; }
uint32_t Um982UartBsp::rearm_error_count() const { return rearm_error_count_; }

void Um982UartBsp::on_rx_complete(UART_HandleTypeDef *uart)
{
    if (uart != uart_) return;
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

void Um982UartBsp::on_uart_error(UART_HandleTypeDef *uart)
{
    if (uart != uart_) return;
    ++uart_error_count_;
    armed_ = (uart_->RxState != HAL_UART_STATE_READY);
}

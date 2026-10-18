#include "../../TerraMind_FW/BSP/imu_uart_bsp.h"
#include "../../TerraMind_FW/BSP/um982_uart_bsp.h"
#include <cassert>
static uint32_t mask=0,arms=0;static bool fail=false;
extern "C" uint32_t __get_PRIMASK() { return mask; }
extern "C" void __disable_irq() { mask=1; }
extern "C" void __set_PRIMASK(uint32_t value) { mask=value; }
extern "C" uint32_t TimeSync_Counter32() { return 12345; }
extern "C" HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart,uint8_t *buffer,uint16_t n) {
    assert(mask==1 && n==1);++arms;
    if(fail) { fail=false;return HAL_ERROR; }
    if(uart->RxState!=HAL_UART_STATE_READY)return HAL_BUSY;
    uart->buffer=buffer;uart->RxState=HAL_UART_STATE_BUSY_RX;return HAL_OK;
}
template<class Bus> void test(Bus &bus) {
    UART_HandleTypeDef uart={HAL_UART_STATE_READY,nullptr};assert(bus.start(&uart));assert(mask==0);
    auto before=arms;bus.service();assert(arms==before);
    // Reproduce the hardware failure: HAL receive is READY while software's
    // armed flag still says true. service must inspect actual HAL state.
    uart.RxState=HAL_UART_STATE_READY;bus.service();assert(arms==before+1&&uart.RxState==HAL_UART_STATE_BUSY_RX);
    *uart.buffer=0x80;uart.RxState=HAL_UART_STATE_READY;bus.on_rx_complete(&uart);
    uint8_t b=0;uint32_t t=0;assert(bus.read_timed(&b,&t,1)==1&&b==0x80&&t==12345);
    uart.RxState=HAL_UART_STATE_READY;bus.on_uart_error(&uart);fail=true;bus.service();
    assert(bus.rearm_error_count()==1 && mask==0);bus.service();assert(uart.RxState==HAL_UART_STATE_BUSY_RX);
    assert(bus.uart_error_count()==1);
}
int main() { test(ImuUartBsp::instance());test(Um982UartBsp::instance()); }

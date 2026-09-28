#include "imu_uart_bsp.h"
#include "um982_uart_bsp.h"
#include "monitor_uart6_dma.h"
#include "usart.h"

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart4) ImuUartBsp::instance().on_rx_complete(uart);
    else if (uart == &huart5) Um982UartBsp::instance().on_rx_complete(uart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart4) ImuUartBsp::instance().on_uart_error(uart);
    else if (uart == &huart5) Um982UartBsp::instance().on_uart_error(uart);
    else if (uart == &huart6) MonitorUart6Dma::instance().on_uart_error(uart);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart6) MonitorUart6Dma::instance().on_tx_complete(uart);
}

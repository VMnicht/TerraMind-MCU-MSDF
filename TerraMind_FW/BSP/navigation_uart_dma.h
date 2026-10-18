#ifndef TERRAMIND_NAVIGATION_UART_DMA_H
#define TERRAMIND_NAVIGATION_UART_DMA_H
#include "stm32h7xx_hal.h"
#include <stdint.h>
struct NavigationTxStats { uint32_t formed,sent,replaced,errors,aborts; };
// One task owns submit/service/reset. Interrupts only set event flags.
class NavigationUartDma {
public:
    static NavigationUartDma &instance();
    bool start(UART_HandleTypeDef *uart);
    bool reset();
    void submit(const uint8_t *packet);
    void service(uint32_t now_ms);
    void complete(UART_HandleTypeDef *uart);
    void error(UART_HandleTypeDef *uart);
    NavigationTxStats stats() const { return stats_; }
private:
    UART_HandleTypeDef *uart_=nullptr;
    alignas(32) uint8_t slots_[2][96]={};
    int active_=-1,pending_=-1;
    volatile uint32_t done_=0,failed_=0;
    uint32_t started_=0;
    NavigationTxStats stats_={};
};
#endif

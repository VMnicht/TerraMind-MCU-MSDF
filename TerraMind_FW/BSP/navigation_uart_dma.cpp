#include "navigation_uart_dma.h"
#include <cstring>
NavigationUartDma &NavigationUartDma::instance() { static NavigationUartDma tx;return tx; }
bool NavigationUartDma::start(UART_HandleTypeDef *uart) {
    if(!uart || !uart->hdmatx || uart_)return false;
#if defined(__arm__) || defined(__ARM_ARCH)
    // DMA1 cannot reach DTCM. Fail visibly if linker placement changes.
    const uintptr_t address=reinterpret_cast<uintptr_t>(slots_);
    if(address<0x24000000u || address+sizeof(slots_)>0x24080000u)return false;
#endif
    uart_=uart;return true;
}
bool NavigationUartDma::reset() {
    if(uart_ && HAL_UART_AbortTransmit(uart_)!=HAL_OK)return false;
    active_=pending_=-1;done_=failed_=0;return true;
}
void NavigationUartDma::submit(const uint8_t *packet) {
    ++stats_.formed;
    if(pending_>=0)++stats_.replaced;
    else pending_=active_==0?1:0;
    std::memcpy(slots_[pending_],packet,80);
}
void NavigationUartDma::service(uint32_t now) {
    if(!uart_)return;
    if(failed_ || (active_>=0 && now-started_>50u && !done_)) {
        // Abort must finish before either slot can be reused.
        if(HAL_UART_AbortTransmit(uart_)!=HAL_OK)return;
        ++stats_.errors;++stats_.aborts;active_=-1;done_=failed_=0;
    }
    if(done_) { done_=0;if(active_>=0)++stats_.sent;active_=-1; }
    if(active_>=0 || pending_<0)return;
    active_=pending_;pending_=-1;started_=now;
#if defined(__arm__) || defined(__ARM_ARCH)
    if(SCB->CCR&SCB_CCR_DC_Msk)SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t *>(slots_[active_]),96);
    __DSB();
#endif
    if(HAL_UART_Transmit_DMA(uart_,slots_[active_],80)!=HAL_OK)failed_=1;
}
void NavigationUartDma::complete(UART_HandleTypeDef *uart) { if(uart==uart_)done_=1; }
void NavigationUartDma::error(UART_HandleTypeDef *uart) { if(uart==uart_)failed_=1; }

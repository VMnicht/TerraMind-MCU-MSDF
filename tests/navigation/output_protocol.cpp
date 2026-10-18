#include "nav_output_protocol.h"
#include "../../TerraMind_FW/BSP/navigation_uart_dma.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <iostream>
static uint8_t *active=nullptr;static uint8_t snapshot[80];static bool abort_ok=true;
static UART_HandleTypeDef uart={};
extern "C" HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *,uint8_t *p,uint16_t n) {
    assert(n==80 && active==nullptr);active=p;std::memcpy(snapshot,p,80);return HAL_OK;
}
extern "C" HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *) {
    if(!abort_ok)return HAL_ERROR;active=nullptr;return HAL_OK;
}
int main() {
    assert(NavWire_Crc(reinterpret_cast<const uint8_t *>("123456789"),9)==0x29b1);
    NavOutput out={};out.state_time_s=1000;out.last_gnss_time_s=1000;
    out.state.position[0]=.5;out.state.position[1]=2;out.state.position[2]=20;
    out.state.velocity[1]=1;out.state.attitude[2]=3.141592653589793/2;
    out.quaternion_bn[0]=out.quaternion_bn[3]=std::sqrt(.5);
    uint8_t packet[80];NavWire_Encode(packet,&out,1,0x95,1000.2);
    const uint8_t golden[]={0xaa,0x55,0x01,0x9d,0x01,0,0,0,0,0,0,0,0,0x40,0x8f,0x40,
        0,0,0,0,0,0,0xe0,0x3f,0,0,0,0,0,0,0,0x40,0,0,0xa0,0x41,0,0,0,0,
        0,0,0x80,0x3f,0,0,0,0,0,0,0,0,0,0,0,0,0xdb,0x0f,0xc9,0x3f,0xf3,4,0x35,0x3f,
        0,0,0,0,0,0,0,0,0xf3,4,0x35,0x3f,0xc8,0,0x3d,0x23};
    static_assert(sizeof(golden)==80,"");assert(std::memcmp(golden,packet,80)==0);
    NavWire_Encode(packet,&out,2,NAV_WIRE_VALID|NAV_WIRE_FAULT,1001);
    assert(packet[3]==NAV_WIRE_FAULT && packet[76]==0xe8 && packet[77]==3);
    NavWire_Encode(packet,nullptr,0,NAV_WIRE_VALID,0);assert(!(packet[3]&1)&&packet[76]==255&&packet[77]==255);
    out.quaternion_bn[0]=std::numeric_limits<double>::quiet_NaN();
    NavWire_Encode(packet,&out,3,1,1001);assert(!(packet[3]&1));
    out.quaternion_bn[0]=std::sqrt(.5);
    auto &tx=NavigationUartDma::instance();uart.hdmatx=&uart;assert(tx.start(&uart));
    tx.submit(golden);tx.service(0);assert(active);
    tx.submit(packet);packet[4]=42;tx.submit(packet);
    assert(std::memcmp(active,snapshot,80)==0);assert(tx.stats().replaced==1);
    active=nullptr;tx.complete(&uart);tx.service(2);assert(active[4]==42);
    tx.error(&uart);abort_ok=false;tx.service(3);assert(active); // No reuse before DMA stops.
    packet[4]=43;tx.submit(packet);assert(active[4]==42);
    abort_ok=true;tx.service(4);assert(active[4]==43 && tx.stats().errors==1);
    tx.service(60);assert(!active && tx.stats().aborts==2); // Stuck-transfer timeout.
    tx.submit(packet);tx.service(61);assert(tx.reset() && !active);
    std::cout<<"Golden frame, CRC, finite/fault handling, immutable DMA, replacement, abort and timeout passed\n";
}

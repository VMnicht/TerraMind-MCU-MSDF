#include "../TerraMind_FW/App/app_main.h"
#include "../TerraMind_FW/App/monitor_app.h"
#include "../TerraMind_FW/App/um982_app.h"
#include "../TerraMind_FW/BSP/imu_uart_bsp.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

UART_HandleTypeDef huart4={},huart6={};
static std::vector<uint8_t> input;
static std::string output;
static uint32_t sequence=0,ticks=0,matches=0,edges=0;
ImuUartBsp::ImuUartBsp() {}
ImuUartBsp &ImuUartBsp::instance() {static ImuUartBsp instance;return instance;}
bool ImuUartBsp::start(UART_HandleTypeDef *) {return true;}
void ImuUartBsp::service() {}
size_t ImuUartBsp::read(uint8_t *,size_t) {auto size=input.size();input.clear();return size;}
size_t ImuUartBsp::read_timed(uint8_t *bytes,uint32_t *times,size_t capacity) {
    const size_t n=input.size();assert(n<=capacity);
    if(n)std::memcpy(bytes,input.data(),n);
    for(size_t i=0;i<n;++i)times[i]=ticks+6000;
    input.clear();return n;
}
uint32_t ImuUartBsp::overflow_count() const {return 0;}
uint32_t ImuUartBsp::uart_error_count() const {return 0;}
uint32_t ImuUartBsp::rearm_error_count() const {return 0;}
extern "C" uint32_t HAL_GetTick() {return ticks/4000;}
extern "C" uint64_t TimeSync_ExpandCounter(uint32_t counter) {return counter;}
extern "C" uint8_t TimeSync_MatchDrdy(uint32_t,TimeSyncEdge *edge) {
    ++matches;*edge={sequence,HAL_GetTick(),ticks,0};return 1;
}
extern "C" void TimeSync_RecordUnmatched() {}
extern "C" uint8_t TimeSync_PopDrdy(TimeSyncEdge *edge) {
    if(!edges)return 0;--edges;*edge={};return 1;
}
extern "C" uint8_t TimeSync_PopPps(TimeSyncEdge *) {return 0;}
extern "C" void TimeSync_GetStats(TimeSyncStats *stats) {*stats={};}
extern "C" uint8_t App_Um982GetPosition(Um982Position *) {return 0;}
extern "C" void App_Um982GetStats(Um982AppStats *stats) {*stats={};}
extern "C" HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart,uint8_t *p,uint16_t n) {
    output.append(reinterpret_cast<char *>(p),n);
    MonitorUart6Dma::instance().on_tx_complete(uart);return HAL_OK;
}
extern "C" HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *) {return HAL_OK;}
static void put16(size_t offset,uint16_t v) {input[offset]=v>>8;input[offset+1]=v&255;}
static void put32(size_t offset,int32_t v) {
    const uint32_t u=static_cast<uint32_t>(v);put16(offset,u>>16);put16(offset+2,u&65535);
}
static void sample() {
    ++sequence;ticks+=20000;++edges;
    input.assign(36,0);input[0]=0x80;input[35]=0x0D;
    put16(3,2634);put32(5,4096000);put32(25,-6400000);
    put16(31,static_cast<uint16_t>(sequence*312+sequence/2));
    uint32_t sum=0;for(size_t i=1;i<33;i+=2)sum+=(uint16_t(input[i])<<8)|input[i+1];
    put16(33,static_cast<uint16_t>(sum));App_ImuStep();
    if(App_ImuCalibrationReady() && !g_monitor_started) assert(Monitor_Init());
    Monitor_Step();
}
int main() {
    huart6.hdmatx=&huart6;assert(App_ImuInit());assert(!Monitor_Init());
    assert(App_ImuGetDeltaCtrl()==0x0008);
    for(unsigned i=0;i<600;++i) {sample();assert(output.empty());assert(edges==0);}
    assert(!App_ImuCalibrationReady());
    sample();assert(App_ImuCalibrationReady());
    assert(output.find("V,2,")==0 && output.find("B,1,")!=std::string::npos);
    assert(output.find("I,")==std::string::npos);
    sample();assert(output.find(",4096000,0,0,0,0,-6400000\r\n")!=std::string::npos);
    assert(matches==sequence); // Shared match: calibration must not consume T's edge twice.
    AppImuCorrected corrected;assert(App_ImuGetCorrected(&corrected));
    assert(std::abs(corrected.delta_angle_rad[0])<1e-12);
    AppImuSample raw;assert(App_ImuGetLatest(&raw));assert(raw.delta_angle_raw[0]==4096000);
    GyroCalibrationResult cal;App_ImuGetCalibration(&cal);assert(cal.samples==600);
    assert(App_ImuSetMode(APP_IMU_MODE_RAW32));App_ImuStep();
    assert(!App_ImuCalibrationReady() && !g_monitor_started);
    const size_t bytes=output.size();Monitor_Step();assert(output.size()==bytes);
}

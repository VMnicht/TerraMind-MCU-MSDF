#include "app_navigation.h"
#include "app_main.h"
#include "um982_app.h"
#include "../Navigation/navigation_live.h"
#include "../BSP/navigation_uart_dma.h"
#include "../BSP/time_sync_capture.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "main.h"

namespace {
// Capture is higher priority than fusion. Critical sections only copy a bounded
// input record; the KF and UART encoding never execute with interrupts masked.
template<class T,unsigned N> class Ingress {
    T data_[N];unsigned head_=0,tail_=0,count_=0;
public:
    bool push(const T &v) {
        taskENTER_CRITICAL();const bool ok=count_<N;
        if(ok) { data_[head_]=v;head_=(head_+1)%N;++count_; }
        taskEXIT_CRITICAL();return ok;
    }
    bool pop(T &v) {
        taskENTER_CRITICAL();const bool ok=count_!=0;
        if(ok) { v=data_[tail_];tail_=(tail_+1)%N;--count_; }
        taskEXIT_CRITICAL();return ok;
    }
};
struct ImuInput { NavImu imu;GyroCalibrationResult cal;bool valid; };
struct GnssInput { Um982Position p;Um982Heading h;double time,arrival;bool position; };
Ingress<ImuInput,128> imus;
Ingress<GnssInput,32> gnss;
NavigationLive live;
volatile uint32_t ingress_fault=0;
StaticTask_t task_cb;
StackType_t task_stack[6144];
double nowSeconds() { return double(TimeSync_NowTicks())/TIME_SYNC_TIMER_HZ; }
uint32_t cycles(void *) { return DWT->CYCCNT; }
void onImu(uint64_t ticks,double dt,const double *angle,const double *velocity,uint8_t valid,const GyroCalibrationResult *cal) {
    ImuInput input={};input.imu.time_s=double(ticks)/TIME_SYNC_TIMER_HZ;input.imu.dt_s=dt;
    input.cal=*cal;input.valid=valid!=0;
    for(unsigned i=0;i<3;++i) {
        const int axis=NAV_VEHICLE_AXES[i],source=axis<0?-axis-1:axis-1;const double sign=axis<0?-1:1;
        input.imu.delta_angle_rad[i]=sign*angle[source]; // Core owns bias compensation, including Earth rotation.
        input.imu.delta_velocity_mps[i]=sign*velocity[source];
    }
    if(!imus.push(input))ingress_fault=1;
}
void onGnss(const Um982Position *p,const Um982Heading *h) {
    GnssInput input={};uint64_t ticks=0;input.arrival=nowSeconds();
    if(p) { input.position=true;input.p=*p;
        input.time=TimeSync_GpsToTicks(p->gps_week,p->gps_tow_ms,&ticks)?double(ticks)/TIME_SYNC_TIMER_HZ:-1;
        if(!gnss.push(input))ingress_fault=2;
    }
    if(h) { input.position=false;input.h=*h;
        input.time=TimeSync_GpsToTicks(h->gps_week,h->gps_tow_ms,&ticks)?double(ticks)/TIME_SYNC_TIMER_HZ:-1;
        if(!gnss.push(input))ingress_fault=2;
    }
}
}
extern "C" {
struct NavigationDebug {
    NavigationLiveStats live;
    NavStats filter;
    NavigationTxStats tx;
    uint32_t stack_free_bytes,packet_sequence,clock_locked;
} g_navigation_debug={};
}
namespace {
void run(void *) {
    live.init();CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;live.cycleClock(cycles);
    auto &tx=NavigationUartDma::instance();if(!tx.start(&huart3))live.fail(105);
    NavOutput last={};bool has_last=false;uint32_t sequence=0;
    double last_frame=-10,last_fresh=-10;uint8_t previous_status=0xff;
    for(;;) {
        GnssInput g;while(gnss.pop(g)) {
            if(g.position)live.position(g.p,g.time,g.arrival);else live.heading(g.h,g.time,g.arrival);
        }
        ImuInput i;while(imus.pop(i))live.imu(i.imu,i.valid,i.cal);
        if(ingress_fault)live.fail(110+ingress_fault);
        NavOutput output;const bool updated=live.step(nowSeconds(),output);
        const double now=nowSeconds();
        if(updated) { last=output;has_last=true;last_fresh=now; }
        TimeSyncClockMap map={};const bool locked=TimeSync_GetClockMap(&map)!=0;
        uint8_t status=live.status(now,locked);
        if(!has_last||now-last_fresh>.1)status&=uint8_t(~NAV_WIRE_VALID);
        if(updated || status!=previous_status || (!(status&NAV_WIRE_VALID)&&now-last_frame>=1)) {
            uint8_t packet[80];NavWire_Encode(packet,has_last?&last:nullptr,sequence++,status,now);
            tx.submit(packet);last_frame=now;previous_status=status;
        }
        tx.service(HAL_GetTick());
        g_navigation_debug.live=live.liveStats();live.stats(&g_navigation_debug.filter);
        g_navigation_debug.tx=tx.stats();g_navigation_debug.packet_sequence=sequence;g_navigation_debug.clock_locked=locked;
        // Stack scanning is expensive; sample once a second.
        static uint32_t last_stack_ms=0;const uint32_t ms=HAL_GetTick();
        if(ms-last_stack_ms>=1000) { last_stack_ms=ms;g_navigation_debug.stack_free_bytes=uxTaskGetStackHighWaterMark(nullptr)*sizeof(StackType_t); }
        osDelay(1);
    }
}
}
extern "C" void AppNavigation_StartTask(void) {
    const osThreadAttr_t attributes={"navigation",0,&task_cb,sizeof(task_cb),task_stack,sizeof(task_stack),osPriorityNormal,0,0};
    if(!osThreadNew(run,nullptr,&attributes))Error_Handler();
}
extern "C" void AppNavigation_ConnectInputs(void) {
    App_ImuSetNavigationCallback(onImu);App_Um982SetNavigationCallback(onGnss);
}

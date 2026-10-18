// Replaces app_tasks.cpp ONLY in the generated benchmark target.
#include "replay_runner.h"
#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "../../TerraMind_FW/Navigation/nav_output_protocol.h"
#include "../../TerraMind_FW/BSP/navigation_uart_dma.h"
#include "usart.h"

struct ReplayPassReport {
    NavStats stats;
    NavOutput last_output;
    double elapsed_s, processing_hz, output_hz, max_publish_interval_s;
    uint32_t stack_free_bytes;
    uint32_t branches_over_5ms;
};
struct ReplayReport {
    uint32_t phase; // 0=boot, 1=fast, 2=realtime, 3=complete, 4=failed
    uint32_t cpu_hz, context_bytes, error;
    uint32_t icache_enabled, dcache_enabled;
    const char *source_sha256;
    ReplayPassReport fast, realtime;
    NavigationTxStats serial;
    uint32_t serial_max_cycles;
    uint8_t last_packet[80];
};
extern "C" { volatile ReplayReport g_nav_replay_report={}; }
static ReplayRunner runner;
static uint32_t cycleClock(void *) {return DWT->CYCCNT;}
static uint64_t cycles64() {
    static uint32_t previous=0;static uint64_t total=0;
    uint32_t current=DWT->CYCCNT;total+=static_cast<uint32_t>(current-previous);previous=current;return total;
}
static uint64_t pass_begin=0;
static double pass_origin=0;
static double publicationClock() {return pass_origin+static_cast<double>(cycles64()-pass_begin)/SystemCoreClock;}
static void stop(NavStatus status) {
    g_nav_replay_report.error=status;g_nav_replay_report.phase=4;
    for(;;) osDelay(1000);
}
static void pass(bool realtime, volatile ReplayPassReport &destination) {
    if(!runner.begin(7,cycleClock)) stop(NAV_INVALID_ARGUMENT);
    const double origin=replay_imu[0].value.time_s;
    const uint64_t begin=cycles64();unsigned step=0;
    pass_begin=begin;pass_origin=origin;
    NavStatus failure=NAV_OK;
    uint64_t last_sequence=0;uint32_t packet_sequence=0;
    while(!runner.done()) {
        const double now=origin+(realtime?static_cast<double>(cycles64()-begin)/SystemCoreClock:step++*.001);
        if(!runner.tick(now,realtime?publicationClock:nullptr)) {failure=runner.status;break;}
        if(realtime) {
            const uint32_t start=DWT->CYCCNT;
            auto &tx=NavigationUartDma::instance();
            if(runner.output.sequence && runner.output.sequence!=last_sequence) {
                last_sequence=runner.output.sequence;uint8_t packet[80];
                // Real captured input replay, explicitly marked SIMULATED.
                NavWire_Encode(packet,&runner.output,packet_sequence++,NAV_WIRE_VALID|NAV_WIRE_SIMULATED|(4u<<5),publicationClock());
                tx.submit(packet);for(unsigned i=0;i<80;++i)g_nav_replay_report.last_packet[i]=packet[i];
            }
            tx.service(HAL_GetTick());const uint32_t elapsed=DWT->CYCCNT-start;
            if(elapsed>g_nav_replay_report.serial_max_cycles)g_nav_replay_report.serial_max_cycles=elapsed;
        }
        // A bound also makes stalled ingress visible instead of running forever.
        if(now>replay_imu[replay_imu_count-1].value.time_s+2) {failure=NAV_TIME_ORDER;break;}
        if(realtime) osDelay(1);
        else (void)cycles64(); // Extend CYCCNT across wrap, even during a slow pass.
    }
    Nav_GetStats(runner.ctx,&runner.stats);
    ReplayPassReport result{};result.stats=runner.stats;Nav_GetLatest(runner.ctx,&result.last_output);
    result.elapsed_s=static_cast<double>(cycles64()-begin)/SystemCoreClock;
    result.processing_hz=runner.stats.imu_processed/result.elapsed_s;
    result.output_hz=runner.stats.outputs>1?(runner.stats.outputs-1)/(runner.last_output_time-runner.first_output_time):0;
    result.max_publish_interval_s=runner.max_output_interval_s;
    result.stack_free_bytes=uxTaskGetStackHighWaterMark(nullptr)*sizeof(StackType_t);
    // Branch maxima in stats identify the exact path exceeding the 5 ms budget.
    for(unsigned b=0;b<4;++b) if(result.stats.branch_max_cycles[b]>SystemCoreClock/200) ++result.branches_over_5ms;
    const unsigned char *src=reinterpret_cast<const unsigned char *>(&result);
    volatile unsigned char *dst=reinterpret_cast<volatile unsigned char *>(&destination);
    for(unsigned i=0;i<sizeof(result);++i) dst[i]=src[i];
    if(failure!=NAV_OK) stop(failure);
}
extern "C" void AppTasks_RunImu(void) {
    SystemCoreClockUpdate();g_nav_replay_report.cpu_hz=SystemCoreClock;
    g_nav_replay_report.context_bytes=Nav_ContextSize();
    g_nav_replay_report.source_sha256=replay_source_sha256;
    g_nav_replay_report.icache_enabled=(SCB->CCR & SCB_CCR_IC_Msk)!=0;
    g_nav_replay_report.dcache_enabled=(SCB->CCR & SCB_CCR_DC_Msk)!=0;
    if(SystemCoreClock!=480000000u) stop(NAV_INVALID_ARGUMENT);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0;DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t before=DWT->CYCCNT;for(volatile unsigned i=0;i<1000;++i) __NOP();
    if(DWT->CYCCNT==before) stop(NAV_INVALID_ARGUMENT);
    g_nav_replay_report.phase=1;pass(false,g_nav_replay_report.fast);
    if(!NavigationUartDma::instance().start(&huart3))stop(NAV_INVALID_ARGUMENT);
    g_nav_replay_report.phase=2;pass(true,g_nav_replay_report.realtime);
    for(unsigned i=0;i<10;++i) { NavigationUartDma::instance().service(HAL_GetTick());osDelay(1); }
    const auto serial=NavigationUartDma::instance().stats();
    g_nav_replay_report.serial.formed=serial.formed;g_nav_replay_report.serial.sent=serial.sent;
    g_nav_replay_report.serial.replaced=serial.replaced;g_nav_replay_report.serial.errors=serial.errors;g_nav_replay_report.serial.aborts=serial.aborts;
    g_nav_replay_report.phase=3;
    for(;;) osDelay(1000);
}

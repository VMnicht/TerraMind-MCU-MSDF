#include "um982_app.h"

#include "../BSP/um982_uart_bsp.h"
#include "../BSP/time_sync_capture.h"
#include "monitor_app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

#include <new>

alignas(Um982Parser) static unsigned char g_um982_storage[sizeof(Um982Parser)];
static Um982Parser *g_um982_parser = NULL;
static bool g_has_position = false;
static bool g_has_heading = false;
static uint32_t g_current_byte_counter=0;
static uint64_t g_last_anchor_epoch=0;
static AppGnssNavigationCallback g_navigation_callback=nullptr;
extern "C" void App_Um982SetNavigationCallback(AppGnssNavigationCallback cb) { g_navigation_callback=cb; }

static void on_message(const Um982Position *p,const Um982Heading *h,void *) {
    const uint64_t epoch=p?uint64_t(p->gps_week)*604800000u+p->gps_tow_ms:0;
    if(p && p->time_valid && p->position_valid && p->gps_tow_ms%1000u==0 && epoch>g_last_anchor_epoch) {
        g_last_anchor_epoch=epoch;
        TimeSyncClockMap map;
        if(TimeSync_BindGpsSecond(p->gps_week,p->gps_tow_ms,
                                  TimeSync_ExpandCounter(g_current_byte_counter),&map))Monitor_OnSyncAnchor(map);
    }
    if(g_navigation_callback)g_navigation_callback(p,h);
}

extern "C" {
Um982Position g_um982_debug_position = {};
Um982Heading g_um982_debug_heading = {};
Um982AppStats g_um982_debug_stats = {};
volatile uint32_t g_um982_debug_flags = 0u;
}

extern "C" uint8_t App_Um982Init(void)
{
    if (g_um982_parser == NULL)
    {
        g_um982_parser = new (g_um982_storage) Um982Parser();
        g_um982_parser->set_callback(on_message,nullptr);
    }
    const bool started = Um982UartBsp::instance().start(&huart5);
    if (started) g_um982_debug_flags |= UM982_DEBUG_UART_INIT_OK;
    return started ? 1u : 0u;
}

extern "C" void App_Um982Step(void)
{
    if (g_um982_parser == NULL) return;
    Um982UartBsp &bus = Um982UartBsp::instance();
    bus.service();
    uint8_t bytes[128];
    uint32_t timer_counters[128];
    size_t length;
    while ((length = bus.read_timed(bytes, timer_counters, sizeof(bytes))) != 0u)
    {
        const uint32_t received_at_ms = HAL_GetTick();
        Monitor_OnGnssBytes(bytes, timer_counters, length, received_at_ms);
        // Deliver every complete message, including multiple epochs drained
        // in one call. GPS/PPS binding uses the UART ISR's final-byte counter.
        for(size_t i=0;i<length;++i) {
            g_current_byte_counter=timer_counters[i];
            g_um982_parser->feed(bytes+i,1,received_at_ms);
        }
    }
    Um982Position position;
    Um982Heading heading;
    const bool has_position = g_um982_parser->get_position(position);
    const bool has_heading = g_um982_parser->get_heading(heading);
    const Um982ParserStats stats = g_um982_parser->get_stats();
    uint32_t flags = g_um982_debug_flags & UM982_DEBUG_UART_INIT_OK;
    if (has_position)
    {
        flags |= UM982_DEBUG_POSITION_SEEN;
        if (position.position_valid) flags |= UM982_DEBUG_POSITION_VALID;
        if (position.velocity_valid) flags |= UM982_DEBUG_VELOCITY_VALID;
        if (position.position_valid && position.fix_type == UM982_FIX_RTK_FIXED)
            flags |= UM982_DEBUG_POSITION_FIXED;
    }
    if (has_heading)
    {
        flags |= UM982_DEBUG_HEADING_SEEN;
        if (heading.heading_valid) flags |= UM982_DEBUG_HEADING_VALID;
        if (heading.heading_valid && heading.fix_type == UM982_FIX_RTK_FIXED)
            flags |= UM982_DEBUG_HEADING_FIXED;
    }
    taskENTER_CRITICAL();
    if (has_position) g_um982_debug_position = position;
    if (has_heading) g_um982_debug_heading = heading;
    g_has_position = has_position;
    g_has_heading = has_heading;
    g_um982_debug_stats.parser = stats;
    g_um982_debug_stats.rx_overflows = bus.overflow_count();
    g_um982_debug_stats.uart_errors = bus.uart_error_count();
    g_um982_debug_stats.rx_rearm_errors = bus.rearm_error_count();
    g_um982_debug_flags = flags;
    taskEXIT_CRITICAL();
}

extern "C" uint8_t App_Um982GetPosition(Um982Position *position)
{
    if (position == NULL) return 0u;
    taskENTER_CRITICAL();
    const bool available = g_has_position;
    if (available) *position = g_um982_debug_position;
    taskEXIT_CRITICAL();
    return available ? 1u : 0u;
}

extern "C" uint8_t App_Um982GetHeading(Um982Heading *heading)
{
    if (heading == NULL) return 0u;
    taskENTER_CRITICAL();
    const bool available = g_has_heading;
    if (available) *heading = g_um982_debug_heading;
    taskEXIT_CRITICAL();
    return available ? 1u : 0u;
}

extern "C" void App_Um982GetStats(Um982AppStats *stats)
{
    if (stats == NULL) return;
    taskENTER_CRITICAL();
    *stats = g_um982_debug_stats;
    taskEXIT_CRITICAL();
}

#include "monitor_app.h"

#include "app_main.h"
#include "um982_app.h"
#include "../BSP/time_sync_capture.h"
#include "usart.h"

#include <stdio.h>

extern "C" {
MonitorTxStats g_monitor_debug_tx = {};
volatile uint32_t g_monitor_format_errors = 0u;
volatile uint32_t g_monitor_started = 0u;
}

bool Monitor_Init()
{
    const bool started = MonitorUart6Dma::instance().start(&huart6);
    g_monitor_started = started ? 1u : 0u;
    if (started)
    {
        static const char version[] = "V,2,921600,4000000\r\n";
        MonitorUart6Dma::instance().enqueue(version, sizeof(version) - 1u);
    }
    return started;
}

void Monitor_Step()
{
    if (g_monitor_started == 0u) return;
    static uint32_t last_status_ms = 0u;
    const uint32_t now = HAL_GetTick();
    MonitorUart6Dma &tx = MonitorUart6Dma::instance();
    TimeSyncEdge edge;
    char capture_record[96];
    while (TimeSync_PopDrdy(&edge))
    {
        const int n = snprintf(capture_record, sizeof(capture_record),
            "D,%lu,%lu,%lu,%lu\r\n",
            static_cast<unsigned long>(edge.sequence),
            static_cast<unsigned long>(edge.mcu_ms),
            static_cast<unsigned long>(edge.ticks >> 32),
            static_cast<unsigned long>(edge.ticks));
        if (n > 0 && static_cast<size_t>(n) < sizeof(capture_record))
            tx.enqueue(capture_record, static_cast<size_t>(n));
        else ++g_monitor_format_errors;
    }
    while (TimeSync_PopPps(&edge))
    {
        const int n = snprintf(capture_record, sizeof(capture_record),
            "P,%lu,%lu,%lu,%lu,%lu\r\n",
            static_cast<unsigned long>(edge.sequence),
            static_cast<unsigned long>(edge.mcu_ms),
            static_cast<unsigned long>(edge.ticks >> 32),
            static_cast<unsigned long>(edge.ticks),
            static_cast<unsigned long>(edge.period_ticks));
        if (n > 0 && static_cast<size_t>(n) < sizeof(capture_record))
            tx.enqueue(capture_record, static_cast<size_t>(n));
        else ++g_monitor_format_errors;
    }
    if (now - last_status_ms >= 1000u)
    {
        last_status_ms = now;
        AppImuStats imu;
        Um982AppStats gnss;
        App_ImuGetStats(&imu);
        App_Um982GetStats(&gnss);
        const MonitorTxStats tx_stats = tx.stats();
        char record[256];
        const int length = snprintf(record, sizeof(record),
            "S,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
            static_cast<unsigned long>(now),
            static_cast<unsigned long>(imu.valid_frames),
            static_cast<unsigned long>(imu.checksum_errors),
            static_cast<unsigned long>(imu.framing_errors),
            static_cast<unsigned long>(imu.count_jumps),
            static_cast<unsigned long>(imu.rx_overflows),
            static_cast<unsigned long>(imu.uart_errors),
            static_cast<unsigned long>(imu.rx_rearm_errors),
            static_cast<unsigned long>(gnss.parser.bestnav_messages),
            static_cast<unsigned long>(gnss.parser.heading_messages),
            static_cast<unsigned long>(gnss.parser.crc_errors),
            static_cast<unsigned long>(gnss.parser.format_errors),
            static_cast<unsigned long>(gnss.parser.line_overflows),
            static_cast<unsigned long>(gnss.rx_overflows),
            static_cast<unsigned long>(gnss.uart_errors),
            static_cast<unsigned long>(gnss.rx_rearm_errors),
            static_cast<unsigned long>(tx_stats.records_dropped),
            static_cast<unsigned long>(tx_stats.bytes_discarded),
            static_cast<unsigned long>(tx_stats.dma_start_errors),
            static_cast<unsigned long>(tx_stats.uart_errors),
            static_cast<unsigned long>(tx_stats.queue_high_water),
            static_cast<unsigned long>(g_monitor_format_errors));
        if (length > 0 && static_cast<size_t>(length) < sizeof(record))
            tx.enqueue(record, static_cast<size_t>(length));
        else
            ++g_monitor_format_errors;
        TimeSyncStats sync;
        TimeSync_GetStats(&sync);
        const int sync_length = snprintf(record, sizeof(record),
            "H,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
            static_cast<unsigned long>(now),
            static_cast<unsigned long>(sync.timer_hz_nominal),
            static_cast<unsigned long>(sync.drdy_captures),
            static_cast<unsigned long>(sync.pps_captures),
            static_cast<unsigned long>(sync.drdy_queue_drops),
            static_cast<unsigned long>(sync.pps_queue_drops),
            static_cast<unsigned long>(sync.drdy_overcaptures),
            static_cast<unsigned long>(sync.pps_overcaptures),
            static_cast<unsigned long>(sync.imu_unmatched));
        if (sync_length > 0 && static_cast<size_t>(sync_length) < sizeof(record))
            tx.enqueue(record, static_cast<size_t>(sync_length));
        else ++g_monitor_format_errors;
    }
    tx.service();
    g_monitor_debug_tx = tx.stats();
}

void Monitor_OnImuSample(const G365Imu::Sample &sample, void *context)
{
    (void)context;
    if (g_monitor_started == 0u) return;
    const int32_t *first = sample.mode == G365Imu::Mode::Raw32 ?
                           sample.gyro : sample.delta_angle;
    const int32_t *second = sample.mode == G365Imu::Mode::Raw32 ?
                            sample.accel : sample.delta_velocity;
    char record[160];
    const int length = snprintf(record, sizeof(record),
        "I,%lu,%u,%u,%u,%ld,%u,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
        static_cast<unsigned long>(sample.received_at_ms),
        static_cast<unsigned>(sample.mode),
        static_cast<unsigned>(sample.count),
        static_cast<unsigned>(sample.flag),
        static_cast<long>(sample.temperature),
        static_cast<unsigned>(sample.gpio),
        static_cast<long>(first[0]), static_cast<long>(first[1]),
        static_cast<long>(first[2]), static_cast<long>(second[0]),
        static_cast<long>(second[1]), static_cast<long>(second[2]));
    if (length <= 0 || static_cast<size_t>(length) >= sizeof(record))
    {
        ++g_monitor_format_errors;
        return;
    }
    MonitorUart6Dma::instance().enqueue(record, static_cast<size_t>(length));
    const uint64_t uart_ticks = TimeSync_ExpandCounter(sample.uart_end_counter);
    TimeSyncEdge drdy = {};
    const bool matched = uart_ticks != 0u &&
                         TimeSync_MatchDrdy(sample.uart_end_counter, &drdy) != 0u;
    if (uart_ticks == 0u) TimeSync_RecordUnmatched();
    char timing_record[128];
    const int timing_length = snprintf(timing_record, sizeof(timing_record),
        "T,%lu,%u,%lu,%lu,%lu,%lu,%lu\r\n",
        static_cast<unsigned long>(sample.received_at_ms),
        static_cast<unsigned>(sample.count),
        static_cast<unsigned long>(matched ? drdy.sequence : 0u),
        static_cast<unsigned long>(matched ? drdy.ticks >> 32 : 0u),
        static_cast<unsigned long>(matched ? drdy.ticks : 0u),
        static_cast<unsigned long>(uart_ticks >> 32),
        static_cast<unsigned long>(uart_ticks));
    if (timing_length > 0 && static_cast<size_t>(timing_length) < sizeof(timing_record))
        MonitorUart6Dma::instance().enqueue(timing_record,
                                            static_cast<size_t>(timing_length));
    else ++g_monitor_format_errors;
}

void Monitor_OnGnssBytes(const uint8_t *bytes, const uint32_t *timer_counters,
                         size_t length,
                         uint32_t received_at_ms)
{
    if (g_monitor_started == 0u || bytes == NULL || timer_counters == NULL) return;
    static char line[384];
    static size_t used = 0u;
    static bool dropping = false;
    for (size_t i = 0u; i < length; ++i)
    {
        if (bytes[i] == '\n')
        {
            if (!dropping && used != 0u)
            {
                if (line[used - 1u] == '\r') --used;
                line[used] = '\0';
                char record[416];
                const int written = snprintf(record, sizeof(record), "N,%lu,%s\r\n",
                    static_cast<unsigned long>(received_at_ms), line);
                if (written > 0 && static_cast<size_t>(written) < sizeof(record))
                {
                    MonitorUart6Dma::instance().enqueue(record, static_cast<size_t>(written));
                    const uint64_t ticks = TimeSync_ExpandCounter(timer_counters[i]);
                    char timing_record[80];
                    const int n = snprintf(timing_record, sizeof(timing_record),
                        "R,%lu,%lu,%lu\r\n",
                        static_cast<unsigned long>(received_at_ms),
                        static_cast<unsigned long>(ticks >> 32),
                        static_cast<unsigned long>(ticks));
                    if (n > 0 && static_cast<size_t>(n) < sizeof(timing_record))
                        MonitorUart6Dma::instance().enqueue(timing_record,
                                                            static_cast<size_t>(n));
                    else ++g_monitor_format_errors;
                }
                else
                    ++g_monitor_format_errors;
            }
            used = 0u;
            dropping = false;
        }
        else if (!dropping)
        {
            if (used >= sizeof(line) - 1u)
            {
                ++g_monitor_format_errors;
                dropping = true;
                used = 0u;
            }
            else
            {
                line[used++] = static_cast<char>(bytes[i]);
            }
        }
    }
}

void Monitor_OnSyncAnchor(const TimeSyncClockMap &mapping)
{
    if (g_monitor_started == 0u) return;
    char record[160];
    const int length = snprintf(record, sizeof(record),
        "A,%lu,%lu,%u,%lu,%lu,%lu,%lu,%lu,%u,%u\r\n",
        static_cast<unsigned long>(HAL_GetTick()),
        static_cast<unsigned long>(mapping.pps_sequence),
        static_cast<unsigned>(mapping.gps_week),
        static_cast<unsigned long>(mapping.gps_tow_ms),
        static_cast<unsigned long>(mapping.pps_ticks >> 32),
        static_cast<unsigned long>(mapping.pps_ticks),
        static_cast<unsigned long>(mapping.period_ticks),
        static_cast<unsigned long>(mapping.serial_lag_ticks),
        static_cast<unsigned>(mapping.consecutive),
        static_cast<unsigned>(mapping.locked));
    if (length > 0 && static_cast<size_t>(length) < sizeof(record))
        MonitorUart6Dma::instance().enqueue(record, static_cast<size_t>(length));
    else ++g_monitor_format_errors;
}

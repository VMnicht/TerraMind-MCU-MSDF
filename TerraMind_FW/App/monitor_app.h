#ifndef MONITOR_APP_H
#define MONITOR_APP_H

#include "../BSP/monitor_uart6_dma.h"
#include "../Driver/g365_imu.h"
#include "../BSP/time_sync_capture.h"
#include <stddef.h>
#include <stdint.h>

bool Monitor_Init();
void Monitor_Step();
void Monitor_OnImuSample(const G365Imu::Sample &sample, void *context);
void Monitor_OnGnssBytes(const uint8_t *bytes, const uint32_t *timer_counters,
                         size_t length,
                         uint32_t received_at_ms);
void Monitor_OnSyncAnchor(const TimeSyncClockMap &mapping);

extern "C" MonitorTxStats g_monitor_debug_tx;
extern "C" volatile uint32_t g_monitor_format_errors;
extern "C" volatile uint32_t g_monitor_started;

#endif

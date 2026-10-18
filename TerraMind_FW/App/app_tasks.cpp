#include "app_tasks.h"

#include "app_main.h"
#include "um982_app.h"
#include "monitor_app.h"
#include "cmsis_os2.h"
#include "app_navigation.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

extern "C" volatile uint32_t g_boot_stage;
extern "C" {
struct CaptureDebug {
    AppImuStats imu;
    TimeSyncStats timing;
    TimeSyncClockMap clock;
    uint32_t stack_free_bytes;
} g_capture_debug={};
}

extern "C" void AppTasks_RunImu(void)
{
    g_boot_stage = 0xB0070060u;
    AppNavigation_ConnectInputs();
    App_ImuInit();
    App_Um982Init();

    for (;;)
    {
        App_ImuStep();
        App_Um982Step();
        // No USART6 bytes (including the version banner) before calibration.
        if (App_ImuCalibrationReady() && g_monitor_started == 0u) Monitor_Init();
        Monitor_Step();
        static uint32_t last_debug_ms=0;
        const uint32_t now_ms=HAL_GetTick();
        if(now_ms-last_debug_ms>=1000u) {
            last_debug_ms=now_ms;
            App_ImuGetStats(&g_capture_debug.imu);TimeSync_GetStats(&g_capture_debug.timing);
            TimeSync_GetClockMap(&g_capture_debug.clock);
            g_capture_debug.stack_free_bytes=uxTaskGetStackHighWaterMark(nullptr)*sizeof(StackType_t);
        }
        osDelay(1u);
    }
}

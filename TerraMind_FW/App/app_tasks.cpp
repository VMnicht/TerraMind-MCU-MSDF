#include "app_tasks.h"

#include "app_main.h"
#include "um982_app.h"
#include "monitor_app.h"
#include "cmsis_os2.h"

#include <stdint.h>

extern "C" volatile uint32_t g_boot_stage;

extern "C" void AppTasks_RunImu(void)
{
    g_boot_stage = 0xB0070060u;
    Monitor_Init();
    App_ImuInit();
    App_Um982Init();

    for (;;)
    {
        App_ImuStep();
        App_Um982Step();
        Monitor_Step();
        osDelay(1u);
    }
}

#include "app_main.h"

#include "../BSP/imu_uart_bsp.h"
#include "../Driver/g365_imu.h"
#include "monitor_app.h"
#include "usart.h"

#include <new>
#include <string.h>

// Construct only after HAL/peripheral initialization; no IMU code runs before main().
alignas(G365Imu) static unsigned char g_imu_storage[sizeof(G365Imu)];
static G365Imu *g_imu = NULL;
static volatile uint32_t g_requested_imu_mode = APP_IMU_MODE_RAW32;
static volatile uint32_t g_requested_delta_ctrl = 0x00ccu;
extern "C" volatile uint32_t g_imu_debug_stage = 0u;

extern "C" uint8_t App_ImuInit(void)
{
    g_imu_debug_stage = 1u;
    if (g_imu == NULL)
    {
        g_imu = new (g_imu_storage) G365Imu();
    }
    else
    {
        g_imu->reset();
    }
    g_imu->set_mode(static_cast<G365Imu::Mode>(g_requested_imu_mode));
    g_imu->set_delta_ctrl(static_cast<uint16_t>(g_requested_delta_ctrl));
    g_imu->set_sample_callback(Monitor_OnImuSample, NULL);
    g_imu_debug_stage = 2u;
    const bool armed = ImuUartBsp::instance().start(&huart4);
    if (armed)
    {
        g_imu_debug_stage = 3u;
    }
    return armed ? 1u : 0u;
}

extern "C" void App_ImuStep(void)
{
    if (g_imu == NULL)
    {
        return;
    }
    ImuUartBsp &bus = ImuUartBsp::instance();
    bus.service();
    if (static_cast<uint32_t>(g_imu->mode()) != g_requested_imu_mode ||
        g_imu->delta_ctrl() != static_cast<uint16_t>(g_requested_delta_ctrl))
    {
        // Old and new profiles have the same frame length; bytes already in the
        // receive ring must not be interpreted under the newly selected mode.
        uint8_t discarded[64];
        while (bus.read(discarded, sizeof(discarded)) != 0u) {}
        g_imu->set_mode(static_cast<G365Imu::Mode>(g_requested_imu_mode));
        g_imu->set_delta_ctrl(static_cast<uint16_t>(g_requested_delta_ctrl));
        g_imu_debug_stage = 4u;
    }
    if (g_imu_debug_stage < 4u)
    {
        g_imu_debug_stage = 4u;
    }

    uint8_t bytes[64];
    uint32_t timer_counters[64];
    size_t length;
    while ((length = bus.read_timed(bytes, timer_counters, sizeof(bytes))) != 0u)
    {
        g_imu->feed_timed(bytes, timer_counters, length, HAL_GetTick());
    }
    if (g_imu_debug_stage < 5u && g_imu->get_stats().valid_frames != 0u)
    {
        g_imu_debug_stage = 5u;
    }
}

extern "C" uint8_t App_ImuSetMode(AppImuMode mode)
{
    if (mode != APP_IMU_MODE_RAW32 && mode != APP_IMU_MODE_DELTA32)
    {
        return 0u;
    }
    g_requested_imu_mode = static_cast<uint32_t>(mode);
    return 1u;
}

extern "C" void App_ImuSetDeltaCtrl(uint16_t value)
{
    g_requested_delta_ctrl = value;
}

extern "C" void App_ImuSetGlobCmd3(uint16_t value)
{
    App_ImuSetDeltaCtrl(value);
}

extern "C" uint8_t App_ImuGetLatest(AppImuSample *sample)
{
    if (sample == NULL || g_imu == NULL ||
        static_cast<uint32_t>(g_imu->mode()) != g_requested_imu_mode ||
        g_imu->delta_ctrl() != static_cast<uint16_t>(g_requested_delta_ctrl))
    {
        return 0u;
    }
    G365Imu::Sample raw;
    if (!g_imu->get_latest(raw))
    {
        return 0u;
    }
    sample->mode = static_cast<AppImuMode>(raw.mode);
    sample->flag = raw.flag;
    sample->temperature_raw = raw.temperature;
    sample->temperature_c = raw.temperature_c;
    sample->gyro_x_dps = raw.gyro_x_dps;
    sample->gyro_y_dps = raw.gyro_y_dps;
    sample->gyro_z_dps = raw.gyro_z_dps;
    sample->accel_x_g = raw.accel_x_g;
    sample->accel_y_g = raw.accel_y_g;
    sample->accel_z_g = raw.accel_z_g;
    sample->delta_angle_x_deg = raw.delta_angle_x_deg;
    sample->delta_angle_y_deg = raw.delta_angle_y_deg;
    sample->delta_angle_z_deg = raw.delta_angle_z_deg;
    sample->delta_velocity_x_mps = raw.delta_velocity_x_mps;
    sample->delta_velocity_y_mps = raw.delta_velocity_y_mps;
    sample->delta_velocity_z_mps = raw.delta_velocity_z_mps;
    for (unsigned axis = 0u; axis < 3u; ++axis)
    {
        sample->gyro_raw[axis] = raw.gyro[axis];
        sample->accel_raw[axis] = raw.accel[axis];
        sample->delta_angle_raw[axis] = raw.delta_angle[axis];
        sample->delta_velocity_raw[axis] = raw.delta_velocity[axis];
    }
    sample->gpio = raw.gpio;
    sample->count = raw.count;
    sample->received_at_ms = raw.received_at_ms;
    return 1u;
}

extern "C" void App_ImuGetStats(AppImuStats *stats)
{
    if (stats == NULL)
    {
        return;
    }
    if (g_imu == NULL)
    {
        memset(stats, 0, sizeof(*stats));
        return;
    }
    const G365Imu::Stats protocol = g_imu->get_stats();
    stats->valid_frames = protocol.valid_frames;
    stats->checksum_errors = protocol.checksum_errors;
    stats->framing_errors = protocol.framing_errors;
    stats->count_jumps = protocol.count_jumps;
    stats->estimated_missing_frames = protocol.estimated_missing_frames;
    stats->sensor_error_frames = protocol.sensor_error_frames;
    stats->range_over_frames = protocol.range_over_frames;
    ImuUartBsp &bus = ImuUartBsp::instance();
    stats->rx_overflows = bus.overflow_count();
    stats->uart_errors = bus.uart_error_count();
    stats->rx_rearm_errors = bus.rearm_error_count();
}

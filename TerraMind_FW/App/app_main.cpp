#include "app_main.h"
#include "imu_capture_config.h"

#include "../BSP/imu_uart_bsp.h"
#include "../Driver/g365_imu.h"
#include "monitor_app.h"
#include "um982_app.h"
#include "usart.h"

#include <new>
#include <string.h>
#include <cmath>

// Construct only after HAL/peripheral initialization; no IMU code runs before main().
alignas(G365Imu) static unsigned char g_imu_storage[sizeof(G365Imu)];
static G365Imu *g_imu = NULL;
static volatile uint32_t g_requested_imu_mode = IMU_CAPTURE_DEFAULT_MODE;
static volatile uint32_t g_requested_delta_ctrl = IMU_CAPTURE_DELTA_CTRL;
extern "C" volatile uint32_t g_imu_debug_stage = 0u;
static GyroBiasCalibration g_calibration;
static_assert(IMU_GYRO_CALIBRATION_SECONDS>=1.0 && IMU_GYRO_CALIBRATION_SECONDS<=30.0,
              "Startup calibration duration must be between 1 and 30 seconds");
static uint64_t g_previous_drdy_ticks = 0;
static uint32_t g_previous_drdy_sequence = 0;
static uint16_t g_previous_count = 0;
extern "C" {
GyroCalibrationResult g_gyro_calibration_debug = {};
AppImuCorrected g_imu_corrected_debug = {};
}

static void reset_calibration()
{
    Monitor_Pause();
    GyroBiasCalibration::Config config = GyroBiasCalibration::defaults();
    config.duration_s = IMU_GYRO_CALIBRATION_SECONDS;
    g_calibration.reset(config);
    g_previous_drdy_ticks = 0;
    g_previous_drdy_sequence = 0;
    g_gyro_calibration_debug = g_calibration.result();
    g_imu_corrected_debug = AppImuCorrected{};
}

static void on_imu_sample(const G365Imu::Sample &sample, void *)
{
    // Consume the DRDY match exactly once, shared by calibration and T logging.
    const uint64_t uart_ticks = TimeSync_ExpandCounter(sample.uart_end_counter);
    TimeSyncEdge drdy = {};
    const bool matched = uart_ticks != 0 && TimeSync_MatchDrdy(sample.uart_end_counter, &drdy);
    if (uart_ticks == 0) TimeSync_RecordUnmatched();
    const uint16_t step = static_cast<uint16_t>(sample.count - g_previous_count);
    const bool had_previous = g_previous_drdy_ticks != 0;
    const bool continuous = matched && g_previous_drdy_ticks != 0 &&
        drdy.ticks > g_previous_drdy_ticks && drdy.sequence == g_previous_drdy_sequence + 1 &&
        (step == 312 || step == 313);
    const double dt = continuous ? double(drdy.ticks-g_previous_drdy_ticks)/TIME_SYNC_TIMER_HZ : 0;
    g_previous_drdy_ticks = matched ? drdy.ticks : 0;
    g_previous_drdy_sequence = drdy.sequence;
    g_previous_count = sample.count;
    const double rad = 3.14159265358979323846/180;
    double angle[3], velocity[3];
    const uint16_t ctrl = g_imu->delta_ctrl();
    const double angle_scale = rad/(66.0*2000*65536)*(1u<<((ctrl>>4)&15));
    const double velocity_scale = (0.4e-3*9.80665)/(2000*65536)*(1u<<(ctrl&15));
    for (unsigned i=0;i<3;++i) {
        angle[i] = sample.mode == G365Imu::Mode::Delta32 ? sample.delta_angle[i]*angle_scale :
            sample.gyro[i]*(rad/(66.0*65536))*dt;
        velocity[i] = sample.mode == G365Imu::Mode::Delta32 ? sample.delta_velocity[i]*velocity_scale :
            sample.accel[i]*(9.80665/(2500.0*65536))*dt;
    }
    Um982Position position;
    const bool moving = App_Um982GetPosition(&position) && position.velocity_valid &&
        sample.received_at_ms-position.received_at_ms<=500u &&
        (position.horizontal_speed_mps>0.2f || std::abs(position.vertical_speed_mps)>0.2f);
    if (!continuous) {
        // The first matched frame is an expected time seed, not a lost interval.
        if (had_previous || !matched) g_calibration.invalidate(GYRO_CAL_TIMING);
    }
    else g_calibration.push(angle,velocity,dt,sample.temperature_c,(sample.flag&0x0101)==0,moving);
    g_gyro_calibration_debug = g_calibration.result();
    g_imu_corrected_debug.valid = 0;
    if (continuous && dt>=0.004 && dt<=0.006 && (sample.flag&0x0101)==0 && App_ImuCalibrationReady()) {
        for (unsigned i=0;i<3;++i) {
            g_imu_corrected_debug.delta_angle_rad[i] = angle[i]-g_gyro_calibration_debug.stationary_rate[i]*dt;
            g_imu_corrected_debug.delta_velocity_mps[i] = velocity[i];
        }
        g_imu_corrected_debug.dt_s = dt;
        g_imu_corrected_debug.received_at_ms = sample.received_at_ms;
        g_imu_corrected_debug.valid = 1;
    }
    Monitor_OnImuTimedSample(sample,matched ? &drdy : NULL,uart_ticks);
}

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
    reset_calibration();
    g_imu->set_sample_callback(on_imu_sample, NULL);
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
        reset_calibration();
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

extern "C" uint8_t App_ImuCalibrationReady(void) {
    return g_calibration.result().state == GYRO_CAL_READY;
}
extern "C" void App_ImuGetCalibration(GyroCalibrationResult *result) {
    if (result) *result = g_calibration.result();
}
extern "C" uint16_t App_ImuGetDeltaCtrl(void) { return static_cast<uint16_t>(g_requested_delta_ctrl); }
extern "C" AppImuMode App_ImuGetMode(void) { return static_cast<AppImuMode>(g_requested_imu_mode); }
extern "C" uint8_t App_ImuGetCorrected(AppImuCorrected *sample) {
    if (!sample || !g_imu_corrected_debug.valid) return 0;
    *sample=g_imu_corrected_debug;return 1;
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

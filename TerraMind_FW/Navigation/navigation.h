#ifndef TERRAMIND_NAVIGATION_H
#define TERRAMIND_NAVIGATION_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define NAV_STATE_DIM 21u
#define NAV_IMU_CAPACITY 256u
#define NAV_GNSS_CAPACITY 32u
typedef struct NavContext NavContext;
typedef enum {
    NAV_OK=0, NAV_NOT_INITIALIZED, NAV_INVALID_ARGUMENT, NAV_TIME_ORDER,
    NAV_QUEUE_FULL, NAV_LATE_GNSS, NAV_IMU_GAP, NAV_NUMERICAL_FAULT,
    NAV_OBSERVATION_DENSITY
} NavStatus;

// SI units throughout: latitude/longitude and Euler RPY in radians, NED m/s,
// gyro bias rad/s, accel bias m/s^2, scale errors dimensionless. Body is FRD.
typedef struct {
    double position[3], velocity[3], attitude[3];
    double gyro_bias[3], accel_bias[3], gyro_scale[3], accel_scale[3];
} NavState;
typedef struct {
    NavState initial, initial_std; // initial_std.position is NED metres, NOT radians.
    double gyro_arw[3], accel_vrw[3]; // rad/sqrt(s), m/s/sqrt(s)
    double gyro_bias_std[3], accel_bias_std[3], gyro_scale_std[3], accel_scale_std[3];
    double correlation_time_s, antenna_lever_frd_m[3];
    double output_hz;       // 0 disables publication; otherwise [1, 200].
    double buffer_delay_s;  // Fixed lag before processing; [0, 1]. NOT a sensor time correction.
    double max_imu_gap_s;   // (0, 0.1]; gaps fault the stream, never fill missing increments.
} NavConfig;
typedef struct {
    double time_s, dt_s, delta_angle_rad[3], delta_velocity_mps[3];
} NavImu;
typedef struct {
    double time_s, position_rad_m[3], std_ned_m[3];
} NavGnss;
typedef struct {
    uint64_t sequence;
    double state_time_s, publish_time_s, age_s, last_gnss_time_s;
    NavState state;
    double std[NAV_STATE_DIM]; // P,V,attitude,bg,ba,sg,sa; SI units as above.
    uint32_t issue_flags;     // Sticky since initialize; bit (1u << NavStatus).
} NavOutput;
typedef struct {
    uint64_t imu_accepted, imu_processed, gnss_accepted, gnss_updates, outputs;
    uint32_t imu_queue_depth, gnss_queue_depth, imu_queue_peak, gnss_queue_peak;
    uint32_t rejected_inputs, imu_overflows, gnss_overflows, late_gnss;
    uint32_t numerical_faults, skipped_output_slots, issue_flags;
    uint32_t last_process_cycles, max_process_cycles, branch_max_cycles[4];
    uint64_t total_process_cycles, branch_count[4]; // 0=predict, 1/2=endpoint, 3=split.
    double last_state_time_s, last_publish_time_s, max_output_age_s;
    NavStatus last_error;
    uint8_t running, faulted;
} NavStats;
typedef uint32_t (*NavCycleClock)(void *user);

// All calls on one context require a single owner or external serialization.
// No HAL/RTOS/UART, dynamic allocation, global instance, or background activity.
size_t Nav_ContextSize(void);
size_t Nav_ContextAlignment(void);
NavContext *Nav_Construct(void *aligned_memory, size_t bytes);
void Nav_Destroy(NavContext *context);
// G365 vehicle noise / initial-uncertainty profile in SI units. Continual GNSS
// updates remain position-only; no heading / Doppler update is enabled here.
// Initial navigation state and installation geometry still require the caller.
void Nav_DefaultConfig(NavConfig *config);
// Initial state must correspond to seed.time_s; seed is the preceding complete
// IMU interval. Both streams and all clock arguments use the SAME continuous
// seconds axis (unwrap GPS weeks in the adapter). No UART arrival timestamps.
NavStatus Nav_Initialize(NavContext *, const NavConfig *, const NavImu *seed);
NavStatus Nav_PushImu(NavContext *, const NavImu *);
NavStatus Nav_PushGnss(NavContext *, const NavGnss *);
// Caller supplies current measurement-domain time; processes at most max_samples.
// Does not wait, sleep, or write logs. Drain sensor ingress before invoking this.
NavStatus Nav_Process(NavContext *, double now_s, uint32_t max_samples, uint32_t *processed);
NavStatus Nav_SetOutputRate(NavContext *, double hz);
// Returns 1 for a new, due snapshot only. Never repeats a sequence or transmits.
uint8_t Nav_PollOutput(NavContext *, double now_s, NavOutput *);
uint8_t Nav_GetLatest(const NavContext *, NavOutput *);
uint8_t Nav_GetCovariance(const NavContext *, double row_major[NAV_STATE_DIM*NAV_STATE_DIM]);
void Nav_GetStats(const NavContext *, NavStats *);
// Optional wrap-safe cycle timer (e.g. DWT->CYCCNT), enabled explicitly by caller.
void Nav_SetCycleClock(NavContext *, NavCycleClock, void *user);

// Adapter helpers, independent of firmware drivers. mode must be 3 (incremental).
// Axis map: +1/-1=X, +2/-2=Y, +3/-3=Z; must preserve right handedness.
NavStatus Nav_G365Delta(uint8_t mode, uint16_t delta_ctrl, const int8_t axes[3],
    double time_s, double dt_s, const int32_t angle[3], const int32_t velocity[3], NavImu *out);
NavStatus Nav_GnssDegrees(double time_s, double latitude_deg, double longitude_deg,
    double height_msl_m, double undulation_m, const double std_ned_m[3], NavGnss *out);
// Optional initialization helper: converts antenna P/V to IMU P/V using the
// configured lever arm and RPY. omega is body rad/s, antenna_velocity is NED m/s.
NavStatus Nav_InitialFromAntenna(NavConfig *, const NavGnss *,
    const double rpy_rad[3], const double antenna_velocity[3], const double omega_frd[3]);
// Convert a startup stationary mean into sensor bias, preserving Earth rotation.
// Mean/std must already be mapped from sensor XYZ to FRD. Latitude and attitude
// must describe the CALIBRATION pose, which may differ from navigation start.
// Feed original increments afterward; do NOT also subtract the stationary mean.
NavStatus Nav_InitialGyroBiasFromStatic(NavConfig *, double calibration_latitude_rad,
    const double calibration_rpy_rad[3], const double stationary_rate_frd[3],
    const double rate_std_frd[3]);
#ifdef __cplusplus
}
#endif
#endif

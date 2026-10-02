#ifndef TERRAMIND_GYRO_BIAS_CALIBRATION_H
#define TERRAMIND_GYRO_BIAS_CALIBRATION_H
#include <stdint.h>

typedef enum {
    GYRO_CAL_WAITING = 0, GYRO_CAL_COLLECTING = 1, GYRO_CAL_READY = 2
} GyroCalibrationState;
typedef enum {
    GYRO_CAL_NO_ERROR = 0, GYRO_CAL_BAD_SAMPLE, GYRO_CAL_TIMING,
    GYRO_CAL_MOTION, GYRO_CAL_UNSTABLE
} GyroCalibrationReason;
typedef struct {
    GyroCalibrationState state;
    GyroCalibrationReason reason;
    uint32_t samples, restarts;
    double duration_s;
    // Sensor XYZ, rad/s. This includes Earth's rotation, NOT navigation bias.
    double stationary_rate[3], rate_std[3], mean_force[3];
    double temperature_c;
} GyroCalibrationResult;

#ifdef __cplusplus
class GyroBiasCalibration {
public:
    struct Config {
        double duration_s, gyro_std_limit, gyro_drift_limit;
        double force_std_limit, gravity_tolerance, max_rate, temperature_span;
    };
    static Config defaults();
    GyroBiasCalibration();
    bool reset(const Config &config);
    void invalidate(GyroCalibrationReason reason);
    void push(const double angle[3], const double velocity[3], double dt,
              double temperature, bool valid, bool moving);
    const GyroCalibrationResult &result() const { return result_; }
private:
    Config config_;
    GyroCalibrationResult result_;
    double angle_sum_[3], velocity_sum_[3], mean_[6], m2_[6];
    double block_sum_[3], block_min_[3], block_max_[3], block_time_;
    double temp_min_, temp_max_;
    uint32_t blocks_;
    void clear(GyroCalibrationReason reason, uint32_t restarts);
};
#endif
#endif

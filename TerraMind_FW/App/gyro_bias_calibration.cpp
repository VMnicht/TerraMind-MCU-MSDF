#include "gyro_bias_calibration.h"
#include <cmath>
#include <cstring>

namespace { const double rad = 3.14159265358979323846 / 180.0; }
GyroBiasCalibration::Config GyroBiasCalibration::defaults() {
    // Initial acquisition thresholds; tune against actual stationary recordings.
    return {3.0, 0.15*rad, 0.03*rad, 0.03*9.80665, 0.15*9.80665, 5.0*rad, 2.0};
}
GyroBiasCalibration::GyroBiasCalibration() { reset(defaults()); }
bool GyroBiasCalibration::reset(const Config &c) {
    const double v[]={c.duration_s,c.gyro_std_limit,c.gyro_drift_limit,
        c.force_std_limit,c.gravity_tolerance,c.max_rate,c.temperature_span};
    for (double x:v) if (!std::isfinite(x) || x<=0) return false;
    if(c.duration_s<1.0 || c.duration_s>30.0) return false;
    config_=c;clear(GYRO_CAL_NO_ERROR,0);return true;
}
void GyroBiasCalibration::clear(GyroCalibrationReason reason,uint32_t restarts) {
    result_=GyroCalibrationResult{};result_.reason=reason;result_.restarts=restarts;
    std::memset(angle_sum_,0,sizeof(angle_sum_));std::memset(velocity_sum_,0,sizeof(velocity_sum_));
    std::memset(mean_,0,sizeof(mean_));std::memset(m2_,0,sizeof(m2_));
    std::memset(block_sum_,0,sizeof(block_sum_));
    std::memset(block_min_,0,sizeof(block_min_));std::memset(block_max_,0,sizeof(block_max_));
    blocks_=0;block_time_=0;temp_min_=temp_max_=0;
}
void GyroBiasCalibration::invalidate(GyroCalibrationReason reason) {
    if(result_.state!=GYRO_CAL_READY) clear(reason,result_.restarts+1);
}
void GyroBiasCalibration::push(const double angle[3],const double velocity[3],double dt,
                               double temperature,bool valid,bool moving) {
    if(result_.state==GYRO_CAL_READY) return;
    if(!valid || !angle || !velocity || !std::isfinite(temperature)) {
        invalidate(GYRO_CAL_BAD_SAMPLE);return;
    }
    if(!std::isfinite(dt) || dt<0.004 || dt>0.006) {invalidate(GYRO_CAL_TIMING);return;}
    double values[6],norm2=0;
    for(unsigned i=0;i<3;++i) {
        values[i]=angle[i]/dt;values[i+3]=velocity[i]/dt;
        if(!std::isfinite(values[i]) || !std::isfinite(values[i+3])) {
            invalidate(GYRO_CAL_BAD_SAMPLE);return;
        }
        norm2+=values[i+3]*values[i+3];
        if(std::abs(values[i])>config_.max_rate) moving=true;
    }
    if(moving || std::abs(std::sqrt(norm2)-9.80665)>config_.gravity_tolerance) {
        invalidate(GYRO_CAL_MOTION);return;
    }
    if(result_.samples==0) temp_min_=temp_max_=temperature;
    if(temperature<temp_min_) temp_min_=temperature;
    if(temperature>temp_max_) temp_max_=temperature;
    if(temp_max_-temp_min_>config_.temperature_span) {invalidate(GYRO_CAL_UNSTABLE);return;}
    ++result_.samples;result_.duration_s+=dt;block_time_+=dt;
    result_.state=GYRO_CAL_COLLECTING;
    result_.temperature_c+=(temperature-result_.temperature_c)/result_.samples;
    for(unsigned i=0;i<6;++i) {
        const double d=values[i]-mean_[i];mean_[i]+=d/result_.samples;
        m2_[i]+=d*(values[i]-mean_[i]);
    }
    for(unsigned i=0;i<3;++i) {
        angle_sum_[i]+=angle[i];velocity_sum_[i]+=velocity[i];block_sum_[i]+=angle[i];
        result_.stationary_rate[i]=angle_sum_[i]/result_.duration_s;
        result_.mean_force[i]=velocity_sum_[i]/result_.duration_s;
        result_.rate_std[i]=result_.samples>1?std::sqrt(m2_[i]/(result_.samples-1)):0;
    }
    if(block_time_>=0.5-1e-9) {
        for(unsigned i=0;i<3;++i) {
            const double b=block_sum_[i]/block_time_;
            if(blocks_==0 || b<block_min_[i]) block_min_[i]=b;
            if(blocks_==0 || b>block_max_[i]) block_max_[i]=b;
            block_sum_[i]=0;
            if(block_max_[i]-block_min_[i]>config_.gyro_drift_limit) {
                invalidate(GYRO_CAL_UNSTABLE);return;
            }
        }
        block_time_=0;++blocks_;
    }
    if(result_.samples>=20) for(unsigned i=0;i<3;++i) {
        if(result_.rate_std[i]>config_.gyro_std_limit ||
           std::sqrt(m2_[i+3]/(result_.samples-1))>config_.force_std_limit) {
            invalidate(GYRO_CAL_UNSTABLE);return;
        }
    }
    if(result_.duration_s+1e-9>=config_.duration_s &&
       result_.samples>=static_cast<uint32_t>(std::ceil(config_.duration_s/0.006))) {
        result_.state=GYRO_CAL_READY;result_.reason=GYRO_CAL_NO_ERROR;
    }
}

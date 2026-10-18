#ifndef TERRAMIND_NAVIGATION_LIVE_H
#define TERRAMIND_NAVIGATION_LIVE_H
#include "nav_output_protocol.h"
#include "../Driver/um982_parser.h"
#include "../App/gyro_bias_calibration.h"
// Installation used by the validated captures: sensor +Y forward, -X right,
// +Z down; primary antenna 15 cm left; primary->secondary points right.
// Change these measured installation values when moving the unit.
constexpr int8_t NAV_VEHICLE_AXES[3]={2,-1,3};
constexpr double NAV_VEHICLE_LEVER[3]={0,-.15,0};
constexpr double NAV_BASELINE_YAW=1.5707963267948966;
struct NavigationLiveStats {
    uint32_t phase=0, fault=0, rejected_observations=0, initializations=0;
    // phase: 0 calibration, 1 clock/RTK/static alignment, 2 running, 3 fault.
    double baseline_m=0, last_imu_s=-1;
};
class NavigationLive {
public:
    void init();
    void imu(const NavImu &sample,bool valid,const GyroCalibrationResult &calibration);
    void position(const Um982Position &p,double mapped_time_s,double now_s);
    void heading(const Um982Heading &h,double mapped_time_s,double now_s);
    bool step(double now_s,NavOutput &output);
    uint8_t status(double now_s,bool time_locked) const;
    void fail(uint32_t reason);
    const NavigationLiveStats &liveStats() const { return live_; }
    void stats(NavStats *out) const { Nav_GetStats(context_,out); }
    void cycleClock(NavCycleClock clock) { Nav_SetCycleClock(context_,clock,nullptr); }
private:
    alignas(16) unsigned char memory_[65536];
    NavContext *context_=nullptr;
    NavigationLiveStats live_={};
    NavImu history_[96]={};
    unsigned history_count_=0,history_head_=0;
    GyroCalibrationResult calibration_={};
    struct Epoch { uint64_t key=0;NavObservation obs={};bool used=false; } epochs_[8];
    Um982Position position_={};Um982Heading heading_={};
    uint64_t position_key_=0,heading_key_=0;
    double position_time_=-1,heading_time_=-1,position_arrival_=-1;
    uint8_t rtk_=0;
    Epoch *epoch(uint64_t key,double time);
    void tryStart(double now);
};
#endif

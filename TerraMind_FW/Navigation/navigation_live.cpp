#include "navigation_live.h"
#include <algorithm>
#include <cmath>
namespace {
constexpr double rad=3.14159265358979323846/180;
uint64_t key(uint16_t week,uint32_t tow) { return uint64_t(week)*604800000u+tow; }
bool fresh(double time,double now) { return std::isfinite(time)&&time>=0&&now>=time&&now-time<=.18; }
bool goodHeading(const Um982Heading &h) {
    return h.time_valid && h.heading_valid && h.fix_type==UM982_FIX_RTK_FIXED &&
        std::isfinite(h.baseline_m) && h.baseline_m>=.1f && h.baseline_m<=100 &&
        std::isfinite(h.heading_deg) && h.heading_deg>=0 && h.heading_deg<360 &&
        std::isfinite(h.heading_std_deg) && h.heading_std_deg>0 && h.heading_std_deg<=5;
}
bool goodVelocity(const Um982Position &p) {
    return p.velocity_valid && std::isfinite(p.horizontal_speed_mps) && p.horizontal_speed_mps>=0 &&
        p.horizontal_speed_mps<200 && std::isfinite(p.vertical_speed_mps) && std::abs(p.vertical_speed_mps)<100 &&
        std::isfinite(p.track_deg) && p.track_deg>=0 && p.track_deg<360 &&
        p.velocity_latency_s==0 && p.velocity_age_s==0 &&
        p.horizontal_speed_std_mps>0 && p.horizontal_speed_std_mps<=1 &&
        p.vertical_speed_std_mps>0 && p.vertical_speed_std_mps<=2;
}
void velocity(const Um982Position &p,double *v) {
    v[0]=p.horizontal_speed_mps*std::cos(p.track_deg*rad);
    v[1]=p.horizontal_speed_mps*std::sin(p.track_deg*rad);v[2]=-p.vertical_speed_mps;
}
}
void NavigationLive::init() { context_=Nav_Construct(memory_,sizeof(memory_));if(!context_)fail(100); }
void NavigationLive::fail(uint32_t reason) { if(!live_.fault)live_.fault=reason;live_.phase=3; }
void NavigationLive::imu(const NavImu &s,bool valid,const GyroCalibrationResult &cal) {
    if(live_.fault)return;
    if(!valid || cal.state!=GYRO_CAL_READY) {
        history_count_=0;
        if(live_.phase==2)fail(101);else live_.phase=cal.state==GYRO_CAL_READY?1:0;
        return;
    }
    calibration_=cal;live_.last_imu_s=s.time_s;
    if(live_.phase==2) { const auto rc=Nav_PushImu(context_,&s);if(rc!=NAV_OK)fail(200+rc);return; }
    live_.phase=1;history_[history_head_]=s;history_head_=(history_head_+1)%96;
    if(history_count_<96)++history_count_;
}
NavigationLive::Epoch *NavigationLive::epoch(uint64_t k,double time) {
    for(auto &e:epochs_)if(e.used&&e.key==k)return &e;
    for(auto &e:epochs_)if(!e.used) { e={};e.used=true;e.key=k;e.obs.time_s=time;return &e; }
    ++live_.rejected_observations;return nullptr;
}
void NavigationLive::position(const Um982Position &p,double time,double now) {
    const uint64_t k=key(p.gps_week,p.gps_tow_ms);
    if(k<=position_key_)return; // Duplicates/backwards epochs never refresh RTK.
    position_key_=k;position_=p;position_time_=time;position_arrival_=now;
    rtk_=p.position_valid&&p.time_valid ? (p.fix_type==UM982_FIX_OTHER?7:uint8_t(p.fix_type)):0;
    if(!p.time_valid || !fresh(time,now)) { rtk_=0;++live_.rejected_observations;return; }
    auto *e=epoch(k,time);if(!e)return;
    NavGnss g={};const double stds[3]={std::fmax(.03,p.latitude_std_m),std::fmax(.03,p.longitude_std_m),std::fmax(.05,p.height_std_m)};
    if(p.position_valid && p.latitude_std_m>0 && p.longitude_std_m>0 && p.height_std_m>0 &&
       // BESTNAV solution age is not velocity timestamp latency. Real fixed
       // solutions report 1 s here; retain a bounded quality check in addition
       // to the independently PPS-mapped measurement freshness check above.
       std::isfinite(p.solution_age_s) && p.solution_age_s>=0 && p.solution_age_s<=2 &&
       Nav_GnssDegrees(time,p.latitude_deg,p.longitude_deg,p.height_msl_m,p.undulation_m,stds,&g)==NAV_OK) {
        e->obs.fields|=NAV_OBS_POSITION;
        for(unsigned i=0;i<3;++i) { e->obs.position_rad_m[i]=g.position_rad_m[i];e->obs.position_std_m[i]=g.std_ned_m[i]; }
    }
    if(goodVelocity(p)) {
        e->obs.fields|=NAV_OBS_VELOCITY;velocity(p,e->obs.velocity_ned_mps);
        e->obs.velocity_std_mps[0]=e->obs.velocity_std_mps[1]=std::fmax(.05,p.horizontal_speed_std_mps);
        e->obs.velocity_std_mps[2]=std::fmax(.05,p.vertical_speed_std_mps);
    }
}
void NavigationLive::heading(const Um982Heading &h,double time,double now) {
    const uint64_t k=key(h.gps_week,h.gps_tow_ms);if(k<=heading_key_)return;
    heading_key_=k;heading_=h;heading_time_=time;
    if(!goodHeading(h) || !fresh(time,now) || (live_.baseline_m>0 &&
       std::abs(h.baseline_m-live_.baseline_m)>std::fmax(.05,.2*live_.baseline_m))) {
        ++live_.rejected_observations;return;
    }
    auto *e=epoch(k,time);if(!e)return;
    e->obs.fields|=NAV_OBS_HEADING;e->obs.heading_rad=h.heading_deg*rad;
    e->obs.heading_std_rad=std::fmax(.5,h.heading_std_deg)*rad;e->obs.baseline_yaw_frd_rad=NAV_BASELINE_YAW;
}
void NavigationLive::tryStart(double now) {
    if(live_.phase!=1 || history_count_<60 || position_key_!=heading_key_ ||
       !fresh(position_time_,now) || !fresh(heading_time_,now) || !goodHeading(heading_) ||
       !position_.position_valid || position_.fix_type!=UM982_FIX_RTK_FIXED || !goodVelocity(position_) ||
       position_.horizontal_speed_mps>.2 || std::abs(position_.vertical_speed_mps)>.2)return;
    bool usable_position=false;
    for(const auto &e:epochs_)if(e.used&&e.key==position_key_&&
        (e.obs.fields&(NAV_OBS_POSITION|NAV_OBS_VELOCITY))==(NAV_OBS_POSITION|NAV_OBS_VELOCITY))usable_position=true;
    if(!usable_position)return;
    // Keep the vehicle still until alignment completes. Check the most recent
    // 300 ms as well as the completed 3 s calibration; a moved pose must not
    // inherit an old pose. Keep original increments; initialize sensor bias
    // from the stationary mean minus the Earth's rotation projection.
    double mean[3],deviation[3],cal_force[3];
    for(unsigned i=0;i<3;++i) {
        const int axis=NAV_VEHICLE_AXES[i],source=axis<0?-axis-1:axis-1;const double sign=axis<0?-1:1;
        mean[i]=sign*calibration_.stationary_rate[source];deviation[i]=calibration_.rate_std[source];
        cal_force[i]=sign*calibration_.mean_force[source];
    }
    double force[3]={}, duration=0;
    for(unsigned j=0;j<60;++j) {
        const auto &s=history_[(history_head_+95-j)%96];duration+=s.dt_s;
        for(unsigned i=0;i<3;++i) {
            force[i]+=s.delta_velocity_mps[i];
            if(std::abs(s.delta_angle_rad[i]/s.dt_s-mean[i])>.01)return;
        }
    }
    if(duration<.29 || duration>.31)return;
    for(unsigned i=0;i<3;++i) {
        force[i]/=duration;
        if(!std::isfinite(force[i]) || std::abs(force[i]-cal_force[i])>.15)return;
    }
    const double norm=std::sqrt(force[0]*force[0]+force[1]*force[1]+force[2]*force[2]);
    if(std::abs(norm-9.80665)>.3)return;
    NavImu seed={};double best=1e9;
    for(unsigned j=0;j<history_count_;++j) {
        const auto &s=history_[(history_head_+95-j)%96];const double d=std::abs(s.time_s-position_time_);
        if(d<best) { best=d;seed=s; }
    }
    if(best>.003)return;
    NavConfig config;Nav_DefaultConfig(&config);config.max_imu_gap_s=.006;
    for(unsigned i=0;i<3;++i)config.antenna_lever_frd_m[i]=NAV_VEHICLE_LEVER[i];
    double rpy[3]={std::atan2(-force[1],-force[2]),std::atan2(force[0],std::hypot(force[1],force[2])),0};
    const double bx=std::cos(NAV_BASELINE_YAW),by=std::sin(NAV_BASELINE_YAW);
    const double north=std::cos(rpy[1])*bx+std::sin(rpy[1])*std::sin(rpy[0])*by;
    const double east=std::cos(rpy[0])*by;
    if(std::hypot(north,east)<.1)return;
    rpy[2]=heading_.heading_deg*rad-std::atan2(east,north);
    const double stds[3]={std::fmax(.03,position_.latitude_std_m),std::fmax(.03,position_.longitude_std_m),std::fmax(.05,position_.height_std_m)};
    NavGnss g;double v[3];velocity(position_,v);double omega[3]={};
    if(Nav_GnssDegrees(seed.time_s,position_.latitude_deg,position_.longitude_deg,position_.height_msl_m,position_.undulation_m,stds,&g)!=NAV_OK)return;
    // Epoch is within half an IMU period; translate the antenna position to
    // the seed time (vehicle is stationary to within 0.2 m/s at initialization).
    const double dt=seed.time_s-position_time_;
    g.position_rad_m[0]+=v[0]*dt/6378137;
    g.position_rad_m[1]+=v[1]*dt/(6378137*std::cos(g.position_rad_m[0]));g.position_rad_m[2]-=v[2]*dt;
    if(Nav_InitialFromAntenna(&config,&g,rpy,v,omega)!=NAV_OK)return;
    if(Nav_InitialGyroBiasFromStatic(&config,config.initial.position[0],rpy,mean,deviation)!=NAV_OK)return;
    for(unsigned i=0;i<3;++i)config.initial_std.position[i]=std::fmax(config.initial_std.position[i],stds[i]);
    config.initial_std.attitude[2]=std::fmax(config.initial_std.attitude[2],heading_.heading_std_deg*rad);
    const auto rc=Nav_Initialize(context_,&config,&seed);if(rc!=NAV_OK) { fail(300+rc);return; }
    live_.phase=2;live_.baseline_m=heading_.baseline_m;++live_.initializations;
    for(unsigned j=0;j<history_count_;++j) {
        const auto &s=history_[(history_head_+96-history_count_+j)%96];
        if(s.time_s>seed.time_s && Nav_PushImu(context_,&s)!=NAV_OK) { fail(102);return; }
    }
    for(auto &e:epochs_)if(e.used&&e.obs.time_s<=seed.time_s+.001)e.used=false;
    history_count_=0;
}
bool NavigationLive::step(double now,NavOutput &out) {
    if(live_.fault)return false;
    tryStart(now);
    // Keep complete epochs for 170 ms, below the 200 ms processing lag. This
    // merges both NMEA-like ASCII messages without injecting two updates at
    // the same filter epoch. Process oldest first even if messages interleave.
    for(unsigned n=0;n<8;++n) {
        Epoch *oldest=nullptr;
        for(auto &e:epochs_)if(e.used&&(!oldest||e.key<oldest->key))oldest=&e;
        if(!oldest || now-oldest->obs.time_s<.17)break;
        if(live_.phase==2 && oldest->obs.fields) {
            const auto rc=Nav_PushObservation(context_,&oldest->obs);
            if(rc!=NAV_OK)++live_.rejected_observations;
        }
        oldest->used=false;
    }
    if(live_.phase!=2)return false;
    if(now-live_.last_imu_s>.1) { fail(103);return false; }
    uint32_t processed=0;const auto rc=Nav_Process(context_,now,1,&processed);
    if(rc!=NAV_OK) { fail(400+rc);return false; }
    if(!Nav_PollOutput(context_,now,&out))return false;
    if(!NavWire_StateFinite(&out)) { fail(104);return false; }
    return true;
}
uint8_t NavigationLive::status(double now,bool locked) const {
    uint8_t s=locked?NAV_WIRE_TIME_LOCKED:0;
    if(live_.phase==2&&!live_.fault)s|=NAV_WIRE_VALID;
    if(live_.fault)s|=NAV_WIRE_FAULT;
    if(position_arrival_>=0 && now-position_arrival_<=.5 && position_time_>=0 &&
       now>=position_time_ && now-position_time_<=.5)s|=uint8_t(rtk_<<5);
    return s;
}

#include "navigation.h"
#include "core/kf-gins/gi_engine.h"
#include "core/common/earth.h"
#include "core/common/rotation.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>

namespace {
constexpr double pi = 3.141592653589793238462643383279502884;
constexpr double align_error = 0.001;
bool finite3(const double *v) {
    return v && std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
bool nonnegative3(const double *v) { return finite3(v) && v[0]>=0 && v[1]>=0 && v[2]>=0; }
bool validState(const NavState &s, bool standard_deviation) {
    const double *groups[] = {s.position,s.velocity,s.attitude,s.gyro_bias,s.accel_bias,s.gyro_scale,s.accel_scale};
    for (auto v : groups) if (!(standard_deviation ? nonnegative3(v) : finite3(v))) return false;
    return true;
}
bool validRate(double hz) { return std::isfinite(hz) && (hz==0 || (hz>=1 && hz<=200)); }
bool validPosition(const double *v) {
    return finite3(v) && std::abs(v[0]) < pi/2 && std::abs(v[1]) <= pi && std::abs(v[2]) < 1e7;
}
bool validImu(const NavImu &imu) {
    return std::isfinite(imu.time_s) && imu.time_s>=0 && std::isfinite(imu.dt_s) && imu.dt_s>0 &&
           finite3(imu.delta_angle_rad) && finite3(imu.delta_velocity_mps);
}
bool validGnss(const NavGnss &g) {
    return std::isfinite(g.time_s) && g.time_s>=0 && validPosition(g.position_rad_m) &&
           finite3(g.std_ned_m) && g.std_ned_m[0]>0 && g.std_ned_m[1]>0 && g.std_ned_m[2]>0;
}
Eigen::Vector3d vec(const double *v) { return {v[0],v[1],v[2]}; }
void copy3(double *out, const Eigen::Vector3d &v) { for (unsigned i=0;i<3;++i) out[i]=v[i]; }
tmnav::NavState coreState(const NavState &s) {
    tmnav::NavState out;
    out.pos=vec(s.position); out.vel=vec(s.velocity); out.euler=vec(s.attitude);
    out.imuerror.gyrbias=vec(s.gyro_bias); out.imuerror.accbias=vec(s.accel_bias);
    out.imuerror.gyrscale=vec(s.gyro_scale); out.imuerror.accscale=vec(s.accel_scale);
    return out;
}
tmnav::IMU coreImu(const NavImu &s) {
    tmnav::IMU out{};
    out.time=s.time_s; out.dt=s.dt_s;
    out.dtheta=vec(s.delta_angle_rad); out.dvel=vec(s.delta_velocity_mps);
    return out;
}
bool validConfig(const NavConfig &c) {
    if (!validState(c.initial,false) || !validState(c.initial_std,true) || !validPosition(c.initial.position) ||
        !validRate(c.output_hz) || !finite3(c.antenna_lever_frd_m) ||
        !std::isfinite(c.correlation_time_s) || c.correlation_time_s<=0 ||
        !std::isfinite(c.buffer_delay_s) || c.buffer_delay_s<0 || c.buffer_delay_s>1 ||
        !std::isfinite(c.max_imu_gap_s) || c.max_imu_gap_s<=0 || c.max_imu_gap_s>0.1) return false;
    const double *noise[]={c.gyro_arw,c.accel_vrw,c.gyro_bias_std,c.accel_bias_std,c.gyro_scale_std,c.accel_scale_std};
    for(auto v:noise) if(!nonnegative3(v)) return false;
    for(unsigned i=0;i<3;++i)
        if(std::abs(1+c.initial.gyro_scale[i])<1e-12 || std::abs(1+c.initial.accel_scale[i])<1e-12) return false;
    return true;
}
}

struct NavContext {
    alignas(tmnav::GIEngine) unsigned char engine_memory[sizeof(tmnav::GIEngine)];
    tmnav::GIEngine *engine;
    NavConfig config;
    NavImu imu[NAV_IMU_CAPACITY];
    NavGnss gnss[NAV_GNSS_CAPACITY];
    uint32_t imu_head, imu_size, gnss_head, gnss_size;
    double last_imu_time, last_gnss_input, next_publish, last_process_now, last_poll_now;
    uint64_t published_sequence;
    bool deadline_set, has_latest;
    NavOutput latest;
    NavStats stats;
    NavCycleClock cycle_clock;
    void *clock_user;
};

namespace {
NavStatus error(NavContext *c, NavStatus s, bool fault=false) {
    if(c) {
        if(!c->stats.faulted) c->stats.last_error=s;
        c->stats.issue_flags|=1u<<s;
        if(fault) { c->stats.faulted=1; c->stats.running=0; }
    }
    return s;
}
NavStatus rejected(NavContext *c, NavStatus s, bool fault=false) {
    if(c) ++c->stats.rejected_inputs;
    return error(c,s,fault);
}
void snapshot(NavContext *c) {
    const auto state=c->engine->getNavState();
    auto &out=c->latest;
    out.sequence=c->stats.imu_processed;
    out.state_time_s=c->engine->timestamp();
    copy3(out.state.position,state.pos); copy3(out.state.velocity,state.vel); copy3(out.state.attitude,state.euler);
    copy3(out.state.gyro_bias,state.imuerror.gyrbias); copy3(out.state.accel_bias,state.imuerror.accbias);
    copy3(out.state.gyro_scale,state.imuerror.gyrscale); copy3(out.state.accel_scale,state.imuerror.accscale);
    const auto &p=c->engine->getCovariance();
    for(unsigned i=0;i<NAV_STATE_DIM;++i) out.std[i]=std::sqrt(p(i,i));
    out.issue_flags=c->stats.issue_flags;
    c->stats.last_state_time_s=out.state_time_s;
    c->has_latest=true;
}
}

extern "C" size_t Nav_ContextSize(void) { return sizeof(NavContext); }
extern "C" size_t Nav_ContextAlignment(void) { return alignof(NavContext); }
extern "C" NavContext *Nav_Construct(void *memory, size_t bytes) {
    if(!memory || bytes<sizeof(NavContext) || reinterpret_cast<uintptr_t>(memory)%alignof(NavContext)) return nullptr;
    return new(memory) NavContext{};
}
extern "C" void Nav_Destroy(NavContext *c) {
    if(!c) return;
    if(c->engine) c->engine->~GIEngine();
    c->~NavContext();
}
extern "C" void Nav_DefaultConfig(NavConfig *c) {
    if(!c) return;
    *c=NavConfig{};
    c->initial.position[0]=30.5*pi/180; c->initial.position[1]=114*pi/180; c->initial.position[2]=20;
    for(unsigned i=0;i<3;++i) {
        c->initial_std.position[i]=i==2?0.2:0.1;
        c->initial_std.velocity[i]=0.05;
        // Same priors / noise as tools/config/kf_gins_g365_vehicle.yaml,
        // converted from degrees / hours to the public API's SI units.
        c->initial_std.attitude[i]=(i==2?3.0:2.0)*pi/180;
        c->gyro_arw[i]=0.24*pi/180/60; c->accel_vrw[i]=0.5/60;
        c->gyro_bias_std[i]=c->initial_std.gyro_bias[i]=50*pi/180/3600;
        // Startup calibration removes gyro bias only. Allow accel bias to
        // converge from an uncertain initial value without increasing its
        // Gauss-Markov process noise (250 mGal) after initialization.
        c->initial_std.accel_bias[i]=10000e-5;
        c->accel_bias_std[i]=250e-5;
        c->gyro_scale_std[i]=c->initial_std.gyro_scale[i]=1000e-6;
        c->accel_scale_std[i]=c->initial_std.accel_scale[i]=1000e-6;
    }
    c->correlation_time_s=3600;
    // Lever arm intentionally unset: must be measured for the user's installation.
    c->output_hz=100; c->buffer_delay_s=0.2; c->max_imu_gap_s=0.1;
}
extern "C" NavStatus Nav_Initialize(NavContext *c, const NavConfig *config, const NavImu *seed) {
    if(!c || !config || !seed || !validConfig(*config) || !validImu(*seed) || seed->dt_s>config->max_imu_gap_s ||
       config->buffer_delay_s>(NAV_IMU_CAPACITY-2)*seed->dt_s) return rejected(c,NAV_INVALID_ARGUMENT);
    tmnav::GINSOptions opt{};
    opt.initstate=coreState(config->initial); opt.initstate_std=coreState(config->initial_std);
    opt.imunoise.gyr_arw=vec(config->gyro_arw); opt.imunoise.acc_vrw=vec(config->accel_vrw);
    opt.imunoise.gyrbias_std=vec(config->gyro_bias_std); opt.imunoise.accbias_std=vec(config->accel_bias_std);
    opt.imunoise.gyrscale_std=vec(config->gyro_scale_std); opt.imunoise.accscale_std=vec(config->accel_scale_std);
    opt.imunoise.corr_time=config->correlation_time_s; opt.antlever=vec(config->antenna_lever_frd_m);
    if(c->engine) c->engine->~GIEngine();
    c->engine=new(c->engine_memory) tmnav::GIEngine(opt);
    c->engine->addImuData(coreImu(*seed),true);
    c->config=*config;
    c->imu_head=c->imu_size=c->gnss_head=c->gnss_size=0;
    c->last_imu_time=seed->time_s; c->last_gnss_input=-1;
    c->last_process_now=c->last_poll_now=-1;
    c->published_sequence=0; c->deadline_set=c->has_latest=false;
    c->latest=NavOutput{}; c->latest.last_gnss_time_s=-1;
    c->stats=NavStats{}; c->stats.running=1; c->stats.last_state_time_s=seed->time_s;
    return NAV_OK;
}
extern "C" NavStatus Nav_PushImu(NavContext *c, const NavImu *imu) {
    if(!c || !c->engine) return NAV_NOT_INITIALIZED;
    if(c->stats.faulted) return c->stats.last_error;
    if(!imu || !validImu(*imu)) return rejected(c,NAV_INVALID_ARGUMENT);
    const double dt=imu->time_s-c->last_imu_time;
    if(dt<=0) return rejected(c,NAV_TIME_ORDER);
    if(dt>c->config.max_imu_gap_s+1e-9) return rejected(c,NAV_IMU_GAP,true);
    // Permit subtraction roundoff on an absolute GPS-seconds axis as well as
    // the preferred small, locally anchored continuous-seconds axis.
    const double time_roundoff=2*std::numeric_limits<double>::epsilon()*(std::abs(imu->time_s)+std::abs(c->last_imu_time));
    if(std::abs(dt-imu->dt_s)>std::max(time_roundoff,std::max(1e-7,dt*1e-5))) return rejected(c,NAV_INVALID_ARGUMENT);
    if(c->imu_size==NAV_IMU_CAPACITY) { ++c->stats.imu_overflows; return rejected(c,NAV_QUEUE_FULL,true); }
    c->imu[(c->imu_head+c->imu_size)%NAV_IMU_CAPACITY]=*imu;
    ++c->imu_size; ++c->stats.imu_accepted;
    c->stats.imu_queue_peak=std::max(c->stats.imu_queue_peak,c->imu_size);
    c->last_imu_time=imu->time_s;
    return NAV_OK;
}
extern "C" NavStatus Nav_PushGnss(NavContext *c, const NavGnss *g) {
    if(!c || !c->engine) return NAV_NOT_INITIALIZED;
    if(c->stats.faulted) return c->stats.last_error;
    if(!g || !validGnss(*g)) return rejected(c,NAV_INVALID_ARGUMENT);
    if(g->time_s<=c->last_gnss_input) return rejected(c,NAV_TIME_ORDER);
    if(g->time_s<c->engine->timestamp()-align_error) { ++c->stats.late_gnss; return rejected(c,NAV_LATE_GNSS); }
    if(c->gnss_size==NAV_GNSS_CAPACITY) { ++c->stats.gnss_overflows; return rejected(c,NAV_QUEUE_FULL,true); }
    c->gnss[(c->gnss_head+c->gnss_size)%NAV_GNSS_CAPACITY]=*g;
    ++c->gnss_size; ++c->stats.gnss_accepted;
    c->stats.gnss_queue_peak=std::max(c->stats.gnss_queue_peak,c->gnss_size);
    c->last_gnss_input=g->time_s;
    return NAV_OK;
}
extern "C" NavStatus Nav_Process(NavContext *c, double now, uint32_t limit, uint32_t *processed) {
    if(processed) *processed=0;
    if(!c || !c->engine) return NAV_NOT_INITIALIZED;
    if(c->stats.faulted) return c->stats.last_error;
    if(!std::isfinite(now) || now<c->last_process_now || now<c->engine->timestamp()) return error(c,NAV_TIME_ORDER);
    c->last_process_now=now;
    uint32_t count=0;
    while(c->imu_size && count<limit) {
        const NavImu &imu=c->imu[c->imu_head];
        if(imu.time_s>now-c->config.buffer_delay_s+1e-9) break;
        // One observation per IMU interval, matching upstream's update contract.
        const bool update=c->gnss_size && c->engine->updateBranchFor(imu.time_s,c->gnss[c->gnss_head].time_s)!=0;
        if(c->gnss_size && !update && c->gnss[c->gnss_head].time_s<imu.time_s) {
            ++c->stats.late_gnss;
            return error(c,NAV_LATE_GNSS,true);
        }
        if(update && c->gnss_size>1 &&
           c->engine->updateBranchFor(imu.time_s,c->gnss[(c->gnss_head+1)%NAV_GNSS_CAPACITY].time_s)!=0)
            return error(c,NAV_OBSERVATION_DENSITY,true);
        const uint32_t begin=c->cycle_clock?c->cycle_clock(c->clock_user):0;
        if(update) {
            const NavGnss &g=c->gnss[c->gnss_head];
            if(g.time_s<c->engine->timestamp()-align_error) return error(c,NAV_LATE_GNSS,true);
            tmnav::GNSS input{}; input.time=g.time_s; input.blh=vec(g.position_rad_m); input.std=vec(g.std_ned_m);
            c->engine->addGnssData(input);
        }
        c->engine->addImuData(coreImu(imu));
        c->engine->newImuProcess();
        if(!c->engine->healthy()) { ++c->stats.numerical_faults; return error(c,NAV_NUMERICAL_FAULT,true); }
        const unsigned branch=static_cast<unsigned>(c->engine->lastBranch());
        // Floating-point boundary comparisons must not silently consume a GNSS.
        if(update && branch==0) return error(c,NAV_TIME_ORDER,true);
        if(update) {
            c->latest.last_gnss_time_s=c->gnss[c->gnss_head].time_s;
            c->gnss_head=(c->gnss_head+1)%NAV_GNSS_CAPACITY; --c->gnss_size;
            ++c->stats.gnss_updates;
        }
        c->imu_head=(c->imu_head+1)%NAV_IMU_CAPACITY; --c->imu_size;
        ++c->stats.imu_processed; ++count;
        snapshot(c);
        ++c->stats.branch_count[branch];
        if(c->cycle_clock) {
            const uint32_t elapsed=c->cycle_clock(c->clock_user)-begin;
            c->stats.last_process_cycles=elapsed;
            c->stats.max_process_cycles=std::max(c->stats.max_process_cycles,elapsed);
            c->stats.branch_max_cycles[branch]=std::max(c->stats.branch_max_cycles[branch],elapsed);
            c->stats.total_process_cycles+=elapsed;
        }
        if(processed) *processed=count;
    }
    return NAV_OK;
}
extern "C" NavStatus Nav_SetOutputRate(NavContext *c, double hz) {
    if(!c || !c->engine) return NAV_NOT_INITIALIZED;
    if(!validRate(hz)) return error(c,NAV_INVALID_ARGUMENT);
    c->config.output_hz=hz; c->deadline_set=false;
    return NAV_OK;
}
extern "C" uint8_t Nav_GetLatest(const NavContext *c, NavOutput *out) {
    if(!c || !out || !c->has_latest) return 0;
    *out=c->latest; out->issue_flags=c->stats.issue_flags;
    return 1;
}
extern "C" uint8_t Nav_PollOutput(NavContext *c, double now, NavOutput *out) {
    if(!c || !out || !c->engine || c->stats.faulted) return 0;
    if(!std::isfinite(now) || now<c->last_poll_now || now<c->stats.last_state_time_s) {
        error(c,NAV_TIME_ORDER); return 0;
    }
    c->last_poll_now=now;
    if(!c->has_latest || c->config.output_hz==0 || c->published_sequence==c->latest.sequence) return 0;
    if(c->deadline_set && now+1e-9<c->next_publish) return 0;
    const double period=1/c->config.output_hz;
    if(!c->deadline_set) { c->next_publish=now; c->deadline_set=true; }
    const double missed=std::max(0.0,std::floor((now-c->next_publish+1e-9)/period));
    const double total=static_cast<double>(c->stats.skipped_output_slots)+missed;
    c->stats.skipped_output_slots=static_cast<uint32_t>(std::min(total,static_cast<double>(UINT32_MAX)));
    c->next_publish+=(missed+1)*period;
    Nav_GetLatest(c,out);
    out->publish_time_s=now; out->age_s=now-out->state_time_s;
    c->published_sequence=out->sequence;
    ++c->stats.outputs; c->stats.last_publish_time_s=now;
    c->stats.max_output_age_s=std::max(c->stats.max_output_age_s,out->age_s);
    return 1;
}
extern "C" uint8_t Nav_GetCovariance(const NavContext *c, double *out) {
    if(!c || !out || !c->engine || c->stats.faulted) return 0;
    const auto &p=c->engine->getCovariance();
    for(unsigned r=0;r<NAV_STATE_DIM;++r) for(unsigned col=0;col<NAV_STATE_DIM;++col) out[r*NAV_STATE_DIM+col]=p(r,col);
    return 1;
}
extern "C" void Nav_GetStats(const NavContext *c, NavStats *out) {
    if(!out) return;
    *out=c?c->stats:NavStats{};
    if(c) { out->imu_queue_depth=c->imu_size; out->gnss_queue_depth=c->gnss_size; }
}
extern "C" void Nav_SetCycleClock(NavContext *c, NavCycleClock clock, void *user) {
    if(c) { c->cycle_clock=clock; c->clock_user=user; }
}

extern "C" NavStatus Nav_G365Delta(uint8_t mode, uint16_t ctrl, const int8_t axes[3],
    double time, double dt, const int32_t angle[3], const int32_t velocity[3], NavImu *out) {
    if(mode!=3 || !axes || !angle || !velocity || !out) return NAV_INVALID_ARGUMENT;
    unsigned mask=0; int parity=0;
    for(unsigned i=0;i<3;++i) {
        const int axis=std::abs(static_cast<int>(axes[i]));
        if(axis<1 || axis>3 || (mask&(1u<<axis))) return NAV_INVALID_ARGUMENT;
        mask|=1u<<axis;
        if(axes[i]<0) ++parity;
        for(unsigned j=0;j<i;++j) if(std::abs(static_cast<int>(axes[j]))>axis) ++parity;
    }
    if(parity%2) return NAV_INVALID_ARGUMENT;
    NavImu result{}; result.time_s=time; result.dt_s=dt;
    const double sa=(pi/180)*(1.0/66/2000)*(1u<<((ctrl>>4)&15))/65536;
    const double sv=(0.4/1000*9.80665/2000)*(1u<<(ctrl&15))/65536;
    for(unsigned i=0;i<3;++i) {
        const int index=std::abs(static_cast<int>(axes[i]))-1;
        const double sign=axes[i]>0?1:-1;
        result.delta_angle_rad[i]=static_cast<double>(angle[index])*sa*sign;
        result.delta_velocity_mps[i]=static_cast<double>(velocity[index])*sv*sign;
    }
    if(!validImu(result)) return NAV_INVALID_ARGUMENT;
    *out=result; return NAV_OK;
}
extern "C" NavStatus Nav_GnssDegrees(double time, double lat, double lon, double height,
    double undulation, const double stddev[3], NavGnss *out) {
    if(!out || !finite3(stddev) || stddev[0]<=0 || stddev[1]<=0 || stddev[2]<=0) return NAV_INVALID_ARGUMENT;
    NavGnss g{}; g.time_s=time;
    g.position_rad_m[0]=lat*pi/180; g.position_rad_m[1]=lon*pi/180; g.position_rad_m[2]=height+undulation;
    for(unsigned i=0;i<3;++i) g.std_ned_m[i]=std::max(stddev[i],i==2?0.05:0.03);
    if(!validGnss(g)) return NAV_INVALID_ARGUMENT;
    *out=g; return NAV_OK;
}
extern "C" NavStatus Nav_InitialFromAntenna(NavConfig *c, const NavGnss *g,
    const double rpy[3], const double velocity[3], const double omega[3]) {
    if(!c || !g || !validGnss(*g) || !finite3(rpy) || !finite3(velocity) || !finite3(omega) ||
       !finite3(c->antenna_lever_frd_m)) return NAV_INVALID_ARGUMENT;
    const auto rotation=tmnav::Rotation::euler2matrix(vec(rpy));
    const Eigen::Vector3d lever=rotation*vec(c->antenna_lever_frd_m);
    copy3(c->initial.position,vec(g->position_rad_m)-tmnav::Earth::DRi(vec(g->position_rad_m))*lever);
    copy3(c->initial.velocity,vec(velocity)-(rotation*vec(omega)).cross(lever));
    copy3(c->initial.attitude,vec(rpy));
    return NAV_OK;
}

extern "C" NavStatus Nav_InitialGyroBiasFromStatic(NavConfig *c,double lat,
    const double rpy[3],const double mean[3],const double stddev[3]) {
    if(!c || !std::isfinite(lat) || std::abs(lat)>pi/2 || !finite3(rpy) ||
       !finite3(mean) || !nonnegative3(stddev) || !finite3(c->initial.gyro_scale) ||
       !nonnegative3(c->initial_std.gyro_bias)) return NAV_INVALID_ARGUMENT;
    const Eigen::Vector3d earth=tmnav::Rotation::euler2matrix(vec(rpy)).transpose()*tmnav::Earth::iewn(lat);
    for(unsigned i=0;i<3;++i) {
        c->initial.gyro_bias[i]=mean[i]-(1+c->initial.gyro_scale[i])*earth[i];
        // Retain a conservative prior; correlated samples do not justify std/sqrt(N).
        c->initial_std.gyro_bias[i]=std::max(c->initial_std.gyro_bias[i],std::max(stddev[i],0.01*pi/180));
    }
    return NAV_OK;
}

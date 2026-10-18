#include "navigation_live.h"
#include <cassert>
#include <cmath>
#include <iostream>
static NavigationLive live;
int main() {
    live.init();GyroCalibrationResult cal={};cal.state=GYRO_CAL_READY;cal.mean_force[2]=-9.793;
    NavOutput out={};unsigned published=0;double last_time=-1;
    Um982Position p={};p.gps_week=2400;p.time_valid=p.position_valid=p.velocity_valid=1;
    p.fix_type=UM982_FIX_RTK_FIXED;p.latitude_deg=30.5;p.longitude_deg=114;p.height_msl_m=20;
    p.latitude_std_m=p.longitude_std_m=.02f;p.height_std_m=.04f;
    p.horizontal_speed_std_mps=p.vertical_speed_std_mps=.05f;
    p.solution_age_s=1; // Actual UM982 fixed BESTNAV reports 1.000, not zero.
    Um982Heading h={};h.gps_week=p.gps_week;h.time_valid=h.heading_valid=1;
    h.fix_type=UM982_FIX_RTK_FIXED;h.baseline_m=1;h.heading_deg=90;h.heading_std_deg=1;
    for(unsigned ms=1;ms<=3000;++ms) {
        const double now=ms*.001;
        if(ms%5==1 && ms>5) {
            NavImu imu={};imu.time_s=(ms-1)*.001;imu.dt_s=.005;imu.delta_velocity_mps[2]=-9.793*.005;
            live.imu(imu,true,cal);
        }
        if(ms%100==30) {
            // A genuinely old solution must still block initialization.
            p.solution_age_s=ms<500?3.f:1.f;
            p.gps_tow_ms=ms-30;live.position(p,(ms-30)*.001,now);
        }
        if(ms%100==45) { h.gps_tow_ms=ms-45;live.heading(h,(ms-45)*.001,now); }
        if(live.step(now,out)) {
            assert(NavWire_StateFinite(&out));assert(out.state_time_s>last_time);last_time=out.state_time_s;++published;
            assert(out.age_s>=.19 && out.age_s<.22);
        }
        if(ms==500)assert(live.liveStats().phase==1 && published==0);
    }
    assert(live.liveStats().phase==2 && live.liveStats().initializations==1 && published>200);
    NavStats stats;live.stats(&stats);assert(!stats.faulted && stats.velocity_updates>10 && stats.heading_updates>10);
    assert(stats.imu_overflows==0 && stats.late_gnss==0);
    assert(live.status(3,true)&NAV_WIRE_VALID);assert((live.status(3,true)>>5)==4);
    live.position(p,3,3.4); // Same GPS epoch must not refresh arrival / measurement freshness.
    assert((live.status(3.6,true)>>5)==0);
    assert(!live.step(3.6,out));assert(live.liveStats().phase==3);
    assert((live.status(3.6,false)&NAV_WIRE_FAULT) && !(live.status(3.6,false)&NAV_WIRE_VALID));
    std::cout<<"Automatic static alignment, merged position/velocity/heading, 100 Hz outputs, stale RTK and lost IMU passed\n";
}

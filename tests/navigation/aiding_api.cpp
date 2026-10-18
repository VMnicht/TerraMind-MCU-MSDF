#include "fixture.h"
#include "aiding.h" // Independent, unchanged offline implementation.
#include <memory>
#include <limits>
#include <iostream>
#define CHECK(x) do {if(!(x)) throw std::runtime_error(std::string(__FILE__)+":"+std::to_string(__LINE__)+": " #x);} while(0)
using namespace tmnav;
static Vector3d v(const double *x) { return {x[0],x[1],x[2]}; }
static tmnav::NavState state(const ::NavState &s) {
    tmnav::NavState n; n.pos=v(s.position);n.vel=v(s.velocity);n.euler=v(s.attitude);
    n.imuerror.gyrbias=v(s.gyro_bias);n.imuerror.accbias=v(s.accel_bias);
    n.imuerror.gyrscale=v(s.gyro_scale);n.imuerror.accscale=v(s.accel_scale);return n;
}
static GINSOptions options(const NavConfig &c) {
    GINSOptions o{};o.initstate=state(c.initial);o.initstate_std=state(c.initial_std);
    o.imunoise.gyr_arw=v(c.gyro_arw);o.imunoise.acc_vrw=v(c.accel_vrw);
    o.imunoise.gyrbias_std=v(c.gyro_bias_std);o.imunoise.accbias_std=v(c.accel_bias_std);
    o.imunoise.gyrscale_std=v(c.gyro_scale_std);o.imunoise.accscale_std=v(c.accel_scale_std);
    o.imunoise.corr_time=c.correlation_time_s;o.antlever=v(c.antenna_lever_frd_m);return o;
}
static IMU input(const NavImu &s) {IMU i{};i.time=s.time_s;i.dt=s.dt_s;i.dtheta=v(s.delta_angle_rad);i.dvel=v(s.delta_velocity_mps);return i;}
static void compare() {
    for(unsigned fields=1;fields<=7;++fields) for(int branch=1;branch<=3;++branch) {
        auto c=fixture::config();auto seed=fixture::imu(0);auto cfg=options(c);
        std::vector<uint64_t> memory((Nav_ContextSize()+7)/8);
        auto context=Nav_Construct(memory.data(),memory.size()*8);CHECK(context);
        CHECK(Nav_Initialize(context,&c,&seed)==NAV_OK);
        auto ref=std::unique_ptr<GIEngine>(new GIEngine(cfg));ref->addImuData(input(seed),true);AidingStats counts;
        for(unsigned k=1;k<=200;++k) {
            auto im=fixture::imu(k);AidingObservation a;const bool update=k%20==0;
            if(update) {
                NavObservation g{};g.time_s=im.time_s-(branch==1?.005:branch==2?0:.0025);g.fields=fields;
                a.time=g.time_s;a.position=fields&1;a.velocity=fields&2;a.heading=fields&4;
                for(int j=0;j<3;++j) {
                    g.position_rad_m[j]=c.initial.position[j];g.position_std_m[j]=.1;
                    g.velocity_ned_mps[j]=0;g.velocity_std_mps[j]=.1;
                }
                g.heading_rad=c.initial.attitude[2];g.heading_std_rad=.02;
                a.blh=v(g.position_rad_m);a.position_std=v(g.position_std_m);
                a.velocity_ned=v(g.velocity_ned_mps);a.velocity_std=v(g.velocity_std_mps);
                a.heading_rad=g.heading_rad;a.heading_std=g.heading_std_rad;
                CHECK(Nav_PushObservation(context,&g)==NAV_OK);
            }
            CHECK(Nav_PushImu(context,&im)==NAV_OK);uint32_t n=0;
            CHECK(Nav_Process(context,im.time_s,1,&n)==NAV_OK && n==1);
            ref->addImuData(input(im));OfflineAiding::process(*ref,update?&a:nullptr,counts);
            NavOutput out{};CHECK(Nav_GetLatest(context,&out));auto expected=ref->getNavState();
            CHECK((v(out.state.position)-expected.pos).norm()<1e-11);
            CHECK((v(out.state.velocity)-expected.vel).norm()<1e-10);
            CHECK((v(out.state.attitude)-expected.euler).norm()<1e-10);
            double covariance[441];CHECK(Nav_GetCovariance(context,covariance));
            for(int r=0;r<21;++r) for(int j=0;j<21;++j) CHECK(std::abs(covariance[r*21+j]-ref->getCovariance()(r,j))<1e-10);
        }
        NavStats s{};Nav_GetStats(context,&s);CHECK(!s.issue_flags && s.gnss_updates==10);
        CHECK(s.velocity_updates==counts.velocity_used && s.velocity_rejected==counts.velocity_rejected);
        CHECK(s.heading_updates==counts.heading_used && s.heading_rejected==counts.heading_rejected);
        CHECK(s.position_updates==((fields&1)?10u:0u));Nav_Destroy(context);
    }
}
static void validation() {
    auto c=fixture::config();auto seed=fixture::imu(0);
    std::vector<uint64_t> memory((Nav_ContextSize()+7)/8);auto ctx=Nav_Construct(memory.data(),memory.size()*8);
    CHECK(Nav_Initialize(ctx,&c,&seed)==NAV_OK);
    NavObservation o{};o.time_s=seed.time_s+.005;
    CHECK(Nav_PushObservation(ctx,&o)==NAV_INVALID_ARGUMENT);
    o.fields=8;CHECK(Nav_PushObservation(ctx,&o)==NAV_INVALID_ARGUMENT);
    o.fields=NAV_OBS_VELOCITY;CHECK(Nav_PushObservation(ctx,&o)==NAV_INVALID_ARGUMENT);
    for(auto &x:o.velocity_std_mps) x=.01; // API clamps tiny variance.
    o.velocity_ned_mps[0]=std::numeric_limits<double>::quiet_NaN();
    CHECK(Nav_PushObservation(ctx,&o)==NAV_INVALID_ARGUMENT);o.velocity_ned_mps[0]=0;
    o.fields|=NAV_OBS_HEADING;o.heading_std_rad=-1;CHECK(Nav_PushObservation(ctx,&o)==NAV_INVALID_ARGUMENT);
    o.heading_std_rad=.001;o.heading_rad=c.initial.attitude[2];
    CHECK(Nav_PushObservation(ctx,&o)==NAV_OK);CHECK(Nav_PushObservation(ctx,&o)==NAV_TIME_ORDER);
    auto im=fixture::imu(1);CHECK(Nav_PushImu(ctx,&im)==NAV_OK);uint32_t n=0;
    CHECK(Nav_Process(ctx,im.time_s,1,&n)==NAV_OK && n==1);
    NavStats s{};Nav_GetStats(ctx,&s);CHECK(s.velocity_updates==1 && s.heading_updates==1 && !s.faulted);
    Nav_Destroy(ctx);
}
int main() {try {compare();validation();std::cout<<"Board C API: all 7 field combinations / 3 branches match offline states and full covariance\n";}
catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}}

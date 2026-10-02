#include "fixture.h"
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#define CHECK(x) do { if(!(x)) throw std::runtime_error(std::string(__FILE__)+":"+std::to_string(__LINE__)+": " #x); } while(0)
struct Context {
    std::vector<unsigned char> memory;
    NavContext *ptr;
    Context():memory(Nav_ContextSize()+Nav_ContextAlignment()),ptr(nullptr) {
        const auto a=Nav_ContextAlignment();auto address=reinterpret_cast<uintptr_t>(memory.data());
        ptr=Nav_Construct(reinterpret_cast<void *>((address+a-1)/a*a),Nav_ContextSize());CHECK(ptr);
    }
    ~Context(){Nav_Destroy(ptr);}
};
static uint32_t fakeClock(void *user) {
    auto &ticks=*static_cast<uint32_t *>(user);ticks+=100;return ticks;
}
static void compare(const fixture::Inputs &data,const std::string &path) {
    Context c;CHECK(Nav_Initialize(c.ptr,&data.cfg,&data.imu.front())==NAV_OK);
    std::ifstream in(path,std::ios::binary);CHECK(in.good());
    size_t gi=0;double maximum_state=0,maximum_covariance=0;
    for(size_t i=1;i<data.imu.size();++i) {
        if(fixture::due(data,gi,i))
            CHECK(Nav_PushGnss(c.ptr,&data.gnss[gi++])==NAV_OK);
        CHECK(Nav_PushImu(c.ptr,&data.imu[i])==NAV_OK);uint32_t n=0;
        CHECK(Nav_Process(c.ptr,data.imu[i].time_s,1,&n)==NAV_OK);CHECK(n==1);
        NavOutput out{};CHECK(Nav_GetLatest(c.ptr,&out));double covariance[441];CHECK(Nav_GetCovariance(c.ptr,covariance));
        const double *groups[]={out.state.position,out.state.velocity,out.state.attitude,out.state.gyro_bias,
                                out.state.accel_bias,out.state.gyro_scale,out.state.accel_scale};
        for(unsigned g=0;g<7;++g) for(unsigned a=0;a<3;++a) {
            double ref;in.read(reinterpret_cast<char *>(&ref),8);CHECK(in.good());
            const double error=std::abs(groups[g][a]-ref);maximum_state=std::max(maximum_state,error);
            // <0.13 mm horizontal, 0.01 mm vertical; tighter than practical sensor precision.
            const double tolerance=g==0?(a<2?2e-11:1e-5):1e-7*(1+std::abs(ref));
            CHECK(std::isfinite(groups[g][a]) && error<tolerance);
        }
        for(unsigned j=0;j<441;++j) {
            double ref;in.read(reinterpret_cast<char *>(&ref),8);CHECK(in.good());
            const double error=std::abs(covariance[j]-ref);maximum_covariance=std::max(maximum_covariance,error);
            CHECK(std::isfinite(covariance[j]) && error<1e-7*(1+std::abs(ref)));
        }
    }
    CHECK(in.peek()==EOF);
    NavStats stats{};Nav_GetStats(c.ptr,&stats);CHECK(stats.imu_processed==data.imu.size()-1);
    CHECK(stats.gnss_updates==gi);CHECK(stats.issue_flags==0);
    if(!data.delivery_frame.empty())
        for(unsigned b=0;b<4;++b) CHECK(stats.branch_count[b]>0);
    std::cout<<path<<": compared "<<stats.imu_processed<<" states and full 21x21 covariance; max abs "
             <<maximum_state<<", "<<maximum_covariance<<"; branches "<<stats.branch_count[0]<<","<<stats.branch_count[1]
             <<","<<stats.branch_count[2]<<","<<stats.branch_count[3]<<"\n";
}
static void runtimeTests() {
    Context c;auto config=fixture::config();auto seed=fixture::imu(0);
    config.buffer_delay_s=0.1;CHECK(Nav_Initialize(c.ptr,&config,&seed)==NAV_OK);
    auto first=fixture::imu(1);CHECK(Nav_PushImu(c.ptr,&first)==NAV_OK);uint32_t n=77;
    CHECK(Nav_Process(c.ptr,first.time_s,10,&n)==NAV_OK && n==0);
    CHECK(Nav_Process(c.ptr,first.time_s+0.1,10,&n)==NAV_OK && n==1);
    NavOutput delayed{};CHECK(Nav_PollOutput(c.ptr,first.time_s+0.1,&delayed));
    CHECK(std::abs(delayed.age_s-0.1)<1e-10);
    auto data=fixture::synthetic();
    CHECK(Nav_Initialize(c.ptr,&config,&seed)==NAV_OK);
    // GNSS arrives 60 ms late; 100 ms processing delay must preserve every update.
    size_t gi=0;
    for(unsigned i=1;i<=1000;++i) {
        auto sample=fixture::imu(i);
        while(gi<data.gnss.size() && data.gnss[gi].time_s+0.06<=sample.time_s)
            CHECK(Nav_PushGnss(c.ptr,&data.gnss[gi++])==NAV_OK);
        CHECK(Nav_PushImu(c.ptr,&sample)==NAV_OK);
        CHECK(Nav_Process(c.ptr,sample.time_s,8,&n)==NAV_OK);
    }
    NavStats stats{};Nav_GetStats(c.ptr,&stats);CHECK(stats.imu_queue_depth==20);CHECK(stats.issue_flags==0);
    CHECK(Nav_Process(c.ptr,1005.1,100,&n)==NAV_OK);Nav_GetStats(c.ptr,&stats);CHECK(stats.imu_processed==1000);
    CHECK(stats.gnss_updates==gi);
    auto late=data.gnss[gi];late.time_s=1004.99;
    CHECK(Nav_PushGnss(c.ptr,&late)==NAV_LATE_GNSS);
    // Rate changes do not reset the filter; zero disables only output.
    config.buffer_delay_s=0;CHECK(Nav_Initialize(c.ptr,&config,&seed)==NAV_OK);
    unsigned emitted[3]={0,0,0};uint64_t last_sequence=0;
    for(unsigned i=1;i<=600;++i) {
        if(i==201) CHECK(Nav_SetOutputRate(c.ptr,20)==NAV_OK);
        if(i==401) CHECK(Nav_SetOutputRate(c.ptr,0)==NAV_OK);
        auto sample=fixture::imu(i);CHECK(Nav_PushImu(c.ptr,&sample)==NAV_OK);
        CHECK(Nav_Process(c.ptr,sample.time_s,1,&n)==NAV_OK);
        NavOutput out{};
        if(Nav_PollOutput(c.ptr,sample.time_s,&out)) {
            ++emitted[(i-1)/200];CHECK(out.sequence>last_sequence);last_sequence=out.sequence;
            CHECK(out.age_s==0);CHECK(!Nav_PollOutput(c.ptr,sample.time_s,&out));
        }
    }
    CHECK(emitted[0]==100 && emitted[1]==20 && emitted[2]==0);
    Nav_GetStats(c.ptr,&stats);CHECK(stats.imu_processed==600);
    CHECK(Nav_SetOutputRate(c.ptr,201)==NAV_INVALID_ARGUMENT);
    CHECK(Nav_SetOutputRate(c.ptr,std::numeric_limits<double>::quiet_NaN())==NAV_INVALID_ARGUMENT);
    // Invalid inputs, discontinuities and bounded queue overflow must be explicit.
    CHECK(Nav_Initialize(c.ptr,&config,&seed)==NAV_OK);
    CHECK(Nav_PushImu(c.ptr,&seed)==NAV_TIME_ORDER);
    auto bad=first;bad.delta_angle_rad[0]=std::numeric_limits<double>::quiet_NaN();
    CHECK(Nav_PushImu(c.ptr,&bad)==NAV_INVALID_ARGUMENT);
    bad=first;bad.time_s=seed.time_s+0.2;bad.dt_s=0.2;CHECK(Nav_PushImu(c.ptr,&bad)==NAV_IMU_GAP);
    CHECK(Nav_Process(c.ptr,bad.time_s,1,&n)==NAV_IMU_GAP);
    CHECK(Nav_Initialize(c.ptr,&config,&seed)==NAV_OK);
    for(unsigned i=1;i<=NAV_IMU_CAPACITY;++i) {auto s=fixture::imu(i);CHECK(Nav_PushImu(c.ptr,&s)==NAV_OK);}
    auto overflow=fixture::imu(NAV_IMU_CAPACITY+1);CHECK(Nav_PushImu(c.ptr,&overflow)==NAV_QUEUE_FULL);
    Nav_GetStats(c.ptr,&stats);CHECK(stats.faulted && stats.imu_overflows==1);
    CHECK(Nav_Initialize(c.ptr,&config,&seed)==NAV_OK);
    auto g=data.gnss.front();g.time_s=first.time_s-0.002;
    CHECK(Nav_PushGnss(c.ptr,&g)==NAV_OK);g.time_s+=0.0001;CHECK(Nav_PushGnss(c.ptr,&g)==NAV_OK);
    CHECK(Nav_PushImu(c.ptr,&first)==NAV_OK);CHECK(Nav_Process(c.ptr,first.time_s,1,&n)==NAV_OBSERVATION_DENSITY);
    // Continuous time across GPS week boundaries, no modulo reset inside core.
    seed.time_s=604799.999;CHECK(Nav_Initialize(c.ptr,&config,&seed)==NAV_OK);
    first.time_s=604800.004;CHECK(Nav_PushImu(c.ptr,&first)==NAV_OK);
    CHECK(Nav_Process(c.ptr,first.time_s,1,&n)==NAV_OK && n==1);
    // DWT-style unsigned wrap and frame classification are usable without HAL.
    uint32_t ticks=UINT32_MAX-50;Nav_SetCycleClock(c.ptr,fakeClock,&ticks);
    first.time_s+=0.005;CHECK(Nav_PushImu(c.ptr,&first)==NAV_OK);
    CHECK(Nav_Process(c.ptr,first.time_s,1,&n)==NAV_OK);Nav_GetStats(c.ptr,&stats);
    CHECK(stats.last_process_cycles==100 && stats.total_process_cycles==100);
    Nav_SetCycleClock(c.ptr,nullptr,nullptr);
    // Non-integer output rate, no bursts after skipped polling slots, resume.
    seed=fixture::imu(0);config.output_hz=33;
    CHECK(Nav_Initialize(c.ptr,&config,&seed)==NAV_OK);unsigned fractional_count=0;
    for(unsigned i=1;i<=2000;++i) {
        auto sample=fixture::imu(i);CHECK(Nav_PushImu(c.ptr,&sample)==NAV_OK);
        CHECK(Nav_Process(c.ptr,sample.time_s,1,&n)==NAV_OK);
        NavOutput output{};fractional_count+=Nav_PollOutput(c.ptr,sample.time_s,&output);
    }
    CHECK(fractional_count==330);
    CHECK(Nav_SetOutputRate(c.ptr,100)==NAV_OK);
    auto sample=fixture::imu(2001);CHECK(Nav_PushImu(c.ptr,&sample)==NAV_OK);
    CHECK(Nav_Process(c.ptr,sample.time_s,1,&n)==NAV_OK);
    NavOutput output{};CHECK(Nav_PollOutput(c.ptr,sample.time_s,&output));
    sample=fixture::imu(2002);CHECK(Nav_PushImu(c.ptr,&sample)==NAV_OK);
    CHECK(Nav_Process(c.ptr,sample.time_s,1,&n)==NAV_OK);
    CHECK(Nav_PollOutput(c.ptr,sample.time_s+1,&output));CHECK(!Nav_PollOutput(c.ptr,sample.time_s+1,&output));
    Nav_GetStats(c.ptr,&stats);CHECK(stats.skipped_output_slots>=99);
    // Invalid initialization must leave the active state intact.
    auto invalid=config;invalid.correlation_time_s=0;
    CHECK(Nav_Initialize(c.ptr,&invalid,&seed)==NAV_INVALID_ARGUMENT);
    Nav_GetStats(c.ptr,&stats);CHECK(stats.imu_processed==2002);
    // Numerical overflow latches a fault and suppresses subsequent publication.
    CHECK(Nav_Initialize(c.ptr,&config,&seed)==NAV_OK);
    sample=fixture::imu(1);sample.delta_velocity_mps[0]=1e300;
    CHECK(Nav_PushImu(c.ptr,&sample)==NAV_OK);
    CHECK(Nav_Process(c.ptr,sample.time_s,1,&n)==NAV_NUMERICAL_FAULT);
    CHECK(!Nav_PollOutput(c.ptr,sample.time_s,&output));
    CHECK(Nav_SetOutputRate(c.ptr,-1)==NAV_INVALID_ARGUMENT);
    CHECK(Nav_Process(c.ptr,sample.time_s,1,&n)==NAV_NUMERICAL_FAULT);
    seed.time_s=1500000000.0;CHECK(Nav_Initialize(c.ptr,&config,&seed)==NAV_OK);
    first=fixture::imu(1);first.time_s=seed.time_s+0.005;
    CHECK(Nav_PushImu(c.ptr,&first)==NAV_OK);
    CHECK(Nav_Process(c.ptr,first.time_s,1,&n)==NAV_OK && n==1);
    std::cout<<"runtime: delay, 100/20/0/33 Hz, skipped slots, cycle wrap, faults, queue limits and week continuity passed\n";
}
static void adapters() {
    const int8_t right[3]={1,2,3},left[3]={1,2,-3};int32_t angle[3]={INT32_MAX,-123456,77},dv[3]={42,33,-400};NavImu out{};
    CHECK(Nav_G365Delta(2,0xcc,right,1,0.005,angle,dv,&out)==NAV_INVALID_ARGUMENT);
    CHECK(Nav_G365Delta(3,0xcc,left,1,0.005,angle,dv,&out)==NAV_INVALID_ARGUMENT);
    CHECK(Nav_G365Delta(3,0xcc,right,1,0.005,angle,dv,&out)==NAV_OK);
    CHECK(std::abs(out.delta_angle_rad[0]-double(INT32_MAX)*(fixture::pi/180)*(1.0/66/2000)*4096/65536)<1e-14);
    NavGnss g{};double stddev[3]={0.01,0.02,0.03};
    CHECK(Nav_GnssDegrees(1,30,114,20,2.5,stddev,&g)==NAV_OK);CHECK(g.position_rad_m[2]==22.5);
    CHECK(g.std_ned_m[0]==0.03 && g.std_ned_m[2]==0.05);
    auto c=fixture::config();c.antenna_lever_frd_m[0]=1;c.antenna_lever_frd_m[1]=0;c.antenna_lever_frd_m[2]=0.2;
    double zero[3]={0,0,0},omega[3]={0,0,1};
    CHECK(Nav_InitialFromAntenna(&c,&g,zero,zero,omega)==NAV_OK);
    CHECK(c.initial.position[0]<g.position_rad_m[0]);CHECK(std::abs(c.initial.position[2]-22.7)<1e-12);
    CHECK(std::abs(c.initial.velocity[1]+1)<1e-12);
    const double earth=7.2921151467E-5;
    const double stationary[3]={earth*(1+c.initial.gyro_scale[0])+0.001,0.002,-0.003};
    CHECK(Nav_InitialGyroBiasFromStatic(&c,0,zero,stationary,zero)==NAV_OK);
    CHECK(std::abs(c.initial.gyro_bias[0]-0.001)<1e-14);
    CHECK(std::abs(c.initial.gyro_bias[1]-0.002)<1e-14);
    CHECK(std::abs(c.initial.gyro_bias[2]+0.003)<1e-14);
    CHECK(c.initial_std.gyro_bias[0]>0);
    const double east_facing[3]={0,0,fixture::pi/2};
    const double east_mean[3]={0.001,0.002-earth*(1+c.initial.gyro_scale[1]),-0.003};
    CHECK(Nav_InitialGyroBiasFromStatic(&c,0,east_facing,east_mean,zero)==NAV_OK);
    CHECK(std::abs(c.initial.gyro_bias[1]-0.002)<1e-14);
    CHECK(Nav_InitialGyroBiasFromStatic(&c,2,zero,stationary,zero)==NAV_INVALID_ARGUMENT);
    std::cout<<"adapters: mode, handedness, double precision increments, ellipsoid height and lever P/V passed\n";
}
int main(int argc,char **argv) {
    try {
        CHECK(argc==3);std::cout<<"context bytes="<<Nav_ContextSize()<<", alignment="<<Nav_ContextAlignment()<<"\n";
        compare(fixture::synthetic(),std::string(argv[1])+"/synthetic.bin");
        compare(fixture::dataset(argv[2]),std::string(argv[1])+"/dataset.bin");
        runtimeTests();adapters();return 0;
    } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}

#ifndef NAV_TEST_FIXTURE_H
#define NAV_TEST_FIXTURE_H
#include "navigation.h"
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>
namespace fixture {
constexpr double pi=3.14159265358979323846;
inline NavConfig config(bool dataset=false) {
    NavConfig c{};
    c.initial.position[0]=(dataset?30.4447873701:30.5)*pi/180;
    c.initial.position[1]=(dataset?114.4718632047:114)*pi/180;
    c.initial.position[2]=dataset?20.899:20;
    const double attitude[]={0.85421502,-2.03480295,185.70235133};
    const double posstd[]={0.005,0.004,0.008}, velstd[]={0.003,0.004,0.004};
    for(unsigned i=0;i<3;++i) {
        c.initial.attitude[i]=dataset?attitude[i]*pi/180:0.01*(i+1);
        c.initial.gyro_bias[i]=dataset?0:1e-6*(i+1);
        c.initial.accel_bias[i]=dataset?0:1e-4*(i+1);
        c.initial.gyro_scale[i]=dataset?0:2e-5*(i+1);
        c.initial.accel_scale[i]=dataset?0:3e-5*(i+1);
        c.initial_std.position[i]=dataset?posstd[i]:0.1;
        c.initial_std.velocity[i]=dataset?velstd[i]:0.05;
        c.initial_std.attitude[i]=(dataset?(i==2?0.023:0.003):0.5)*pi/180;
        c.gyro_arw[i]=(dataset?0.003:0.24)*pi/180/60;
        c.accel_vrw[i]=(dataset?0.03:0.24)/60;
        c.initial_std.gyro_bias[i]=c.gyro_bias_std[i]=(dataset?0.027:50)*pi/180/3600;
        c.initial_std.accel_bias[i]=c.accel_bias_std[i]=(dataset?15:250)*1e-5;
        c.initial_std.gyro_scale[i]=c.gyro_scale_std[i]=(dataset?300:1000)*1e-6;
        c.initial_std.accel_scale[i]=c.accel_scale_std[i]=(dataset?300:1000)*1e-6;
    }
    c.antenna_lever_frd_m[0]=0.136; c.antenna_lever_frd_m[1]=-0.301; c.antenna_lever_frd_m[2]=-0.184;
    c.correlation_time_s=dataset?14400:3600;
    c.max_imu_gap_s=0.1; c.output_hz=100; c.buffer_delay_s=0;
    return c;
}
inline NavImu imu(unsigned n) {
    NavImu s{}; s.time_s=1000+n*0.005; s.dt_s=0.005;
    s.delta_angle_rad[0]=(6.28e-5+0.001*std::sin(n*0.01))*s.dt_s;
    s.delta_angle_rad[1]=0.0004*std::cos(n*0.017)*s.dt_s;
    s.delta_angle_rad[2]=(-3.70e-5+0.001*std::sin(n*0.003))*s.dt_s;
    s.delta_velocity_mps[0]=0.003*std::sin(n*0.023)*s.dt_s;
    s.delta_velocity_mps[1]=0.002*std::cos(n*0.029)*s.dt_s;
    s.delta_velocity_mps[2]=-9.793*s.dt_s;
    return s;
}
struct Inputs {
    NavConfig cfg;
    std::vector<NavImu> imu;
    std::vector<NavGnss> gnss;
    std::vector<unsigned> delivery_frame;
};
inline bool due(const Inputs &data,size_t gi,size_t frame) {
    return gi<data.gnss.size() && (data.delivery_frame.empty()?
        data.gnss[gi].time_s<=data.imu[frame].time_s+0.001 : data.delivery_frame[gi]<=frame);
}
inline Inputs synthetic() {
    Inputs data; data.cfg=config();
    for(unsigned n=0;n<=6000;++n) data.imu.push_back(imu(n));
    for(unsigned n=20;n<=5980;n+=20) {
        if(n>=2000 && n<2600) continue; // GNSS outage and reacquisition.
        NavGnss g{};
        const unsigned which=(n/20)%3;
        g.time_s=1000+n*0.005+(which==0?0:(which==1?-0.005:-0.0025));
        for(unsigned a=0;a<3;++a) { g.position_rad_m[a]=data.cfg.initial.position[a]; g.std_ned_m[a]=0.03+0.01*a; }
        g.position_rad_m[0]+=1e-8*std::sin(n*0.003);
        g.position_rad_m[1]+=1e-8*std::cos(n*0.003);
        g.position_rad_m[2]+=0.03*std::sin(n*0.004);
        data.gnss.push_back(g);
        data.delivery_frame.push_back(n);
    }
    return data;
}
inline Inputs dataset(const std::string &folder) {
    Inputs data; data.cfg=config(true);
    std::ifstream im(folder+"/Leador-A15.txt"), gn(folder+"/GNSS-RTK.txt");
    if(!im || !gn) throw std::runtime_error("Missing upstream dataset");
    std::string line; double previous=0;
    while(std::getline(im,line) && data.imu.size()<12001) {
        std::istringstream in(line); NavImu s{};
        in>>s.time_s; for(auto &v:s.delta_angle_rad) in>>v; for(auto &v:s.delta_velocity_mps) in>>v;
        if(!in) throw std::runtime_error("Malformed upstream IMU");
        s.dt_s=previous>0?s.time_s-previous:0.005; previous=s.time_s;
        if(s.time_s>=456300) data.imu.push_back(s);
    }
    if(data.imu.size()!=12001) throw std::runtime_error("Upstream dataset too short");
    while(std::getline(gn,line)) {
        std::istringstream in(line); NavGnss g{};
        in>>g.time_s; for(auto &v:g.position_rad_m) in>>v; for(auto &v:g.std_ned_m) in>>v;
        if(!in) throw std::runtime_error("Malformed upstream GNSS");
        if(g.time_s<=data.imu.front().time_s) continue;
        if(g.time_s>data.imu.back().time_s) break;
        g.position_rad_m[0]*=pi/180; g.position_rad_m[1]*=pi/180;
        data.gnss.push_back(g);
    }
    return data;
}
}
#endif

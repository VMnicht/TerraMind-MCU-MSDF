// Separate process: untouched upstream Eigen/dynamic-allocation implementation.
#define NavState NavPublicState
#include "fixture.h"
#undef NavState
#include "kf-gins/gi_engine.h"
#include <iostream>
static Eigen::Vector3d v(const double *a) { return {a[0],a[1],a[2]}; }
static NavState convert(const NavPublicState &s) {
    NavState o{}; o.pos=v(s.position); o.vel=v(s.velocity); o.euler=v(s.attitude);
    o.imuerror.gyrbias=v(s.gyro_bias); o.imuerror.accbias=v(s.accel_bias);
    o.imuerror.gyrscale=v(s.gyro_scale); o.imuerror.accscale=v(s.accel_scale); return o;
}
static IMU imu(const NavImu &s) {
    IMU out{};out.time=s.time_s;out.dt=s.dt_s;out.dtheta=v(s.delta_angle_rad);out.dvel=v(s.delta_velocity_mps);return out;
}
static void generate(const fixture::Inputs &data, const std::string &path) {
    const auto &c=data.cfg;
    GINSOptions o{};o.initstate=convert(c.initial);o.initstate_std=convert(c.initial_std);
    o.imunoise.gyr_arw=v(c.gyro_arw);o.imunoise.acc_vrw=v(c.accel_vrw);
    o.imunoise.gyrbias_std=v(c.gyro_bias_std);o.imunoise.accbias_std=v(c.accel_bias_std);
    o.imunoise.gyrscale_std=v(c.gyro_scale_std);o.imunoise.accscale_std=v(c.accel_scale_std);
    o.imunoise.corr_time=c.correlation_time_s;o.antlever=v(c.antenna_lever_frd_m);
    GIEngine engine(o); engine.addImuData(imu(data.imu.front()),true);
    // Explicit future sentinel initializes upstream's otherwise unset GNSS slot;
    // real observations are injected at the fixture's selected delivery frame.
    if(!data.gnss.empty()) {
        GNSS first{};first.time=data.imu.back().time_s+1;
        first.blh=v(data.gnss.front().position_rad_m);first.std=v(data.gnss.front().std_ned_m);
        engine.addGnssData(first);
    }
    std::ofstream file(path,std::ios::binary);if(!file) throw std::runtime_error("Cannot write reference");
    size_t gi=0;
    for(size_t i=1;i<data.imu.size();++i) {
        if(fixture::due(data,gi,i)) {
            const auto &g=data.gnss[gi++]; GNSS input{};input.time=g.time_s;input.blh=v(g.position_rad_m);input.std=v(g.std_ned_m);
            engine.addGnssData(input);
        }
        engine.addImuData(imu(data.imu[i]));engine.newImuProcess();
        const auto s=engine.getNavState();const auto p=engine.getCovariance();
        const Eigen::Vector3d groups[]={s.pos,s.vel,s.euler,s.imuerror.gyrbias,s.imuerror.accbias,s.imuerror.gyrscale,s.imuerror.accscale};
        for(const auto &group:groups) for(int a=0;a<3;++a) file.write(reinterpret_cast<const char *>(&group[a]),sizeof(double));
        for(int r=0;r<21;++r) for(int col=0;col<21;++col) {double value=p(r,col);file.write(reinterpret_cast<const char *>(&value),sizeof(value));}
    }
    if(!file) throw std::runtime_error("Reference write failed");
    std::cout<<"reference "<<path<<": "<<data.imu.size()-1<<" frames\n";
}
int main(int argc,char **argv) {
    try {
        if(argc!=3) throw std::runtime_error("usage: nav_reference output-directory dataset-directory");
        generate(fixture::synthetic(),std::string(argv[1])+"/synthetic.bin");
        generate(fixture::dataset(argv[2]),std::string(argv[1])+"/dataset.bin");
        return 0;
    } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}

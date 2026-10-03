// Offline-only full 21-state KF-GINS with optional heading / Doppler velocity.
// GPL-3.0-or-later; see KF-GINS/LICENSE.
#include "aiding.h"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>
using namespace tmnav;

static Vector3d vector(const YAML::Node &node, double factor=1, bool nonnegative=false) {
    if(!node.IsSequence() || node.size()!=3) throw std::runtime_error("Expected a 3-vector in YAML");
    Vector3d value;
    for(int i=0;i<3;++i) {
        value[i]=node[i].as<double>()*factor;
        if(!std::isfinite(value[i]) || (nonnegative && value[i]<0))
            throw std::runtime_error("Invalid YAML vector value");
    }
    return value;
}
static GINSOptions config(const YAML::Node &c) {
    GINSOptions o{};
    o.initstate.pos=vector(c["initpos"]);o.initstate.pos.head<2>()*=D2R;
    o.initstate.vel=vector(c["initvel"]);o.initstate.euler=vector(c["initatt"],D2R);
    o.initstate.imuerror.gyrbias=vector(c["initgyrbias"],D2R/3600);
    o.initstate.imuerror.accbias=vector(c["initaccbias"],1e-5);
    o.initstate.imuerror.gyrscale=vector(c["initgyrscale"],1e-6);
    o.initstate.imuerror.accscale=vector(c["initaccscale"],1e-6);
    if((o.initstate.imuerror.gyrscale.array()+1).abs().minCoeff()<1e-6 ||
       (o.initstate.imuerror.accscale.array()+1).abs().minCoeff()<1e-6)
        throw std::runtime_error("Singular IMU scale");
    o.initstate_std.pos=vector(c["initposstd"],1,true);
    o.initstate_std.vel=vector(c["initvelstd"],1,true);
    o.initstate_std.euler=vector(c["initattstd"],D2R,true);
    const auto n=c["imunoise"];
    o.imunoise.gyr_arw=vector(n["arw"],D2R/60,true);
    o.imunoise.acc_vrw=vector(n["vrw"],1.0/60,true);
    o.imunoise.gyrbias_std=vector(n["gbstd"],D2R/3600,true);
    o.imunoise.accbias_std=vector(n["abstd"],1e-5,true);
    o.imunoise.gyrscale_std=vector(n["gsstd"],1e-6,true);
    o.imunoise.accscale_std=vector(n["asstd"],1e-6,true);
    o.imunoise.corr_time=n["corrtime"].as<double>()*3600;
    if(!std::isfinite(o.imunoise.corr_time) || o.imunoise.corr_time<=0)
        throw std::runtime_error("Invalid IMU correlation time");
    o.initstate_std.imuerror.gyrbias=c["initbgstd"]?vector(c["initbgstd"],D2R/3600,true):o.imunoise.gyrbias_std;
    o.initstate_std.imuerror.accbias=c["initbastd"]?vector(c["initbastd"],1e-5,true):o.imunoise.accbias_std;
    o.initstate_std.imuerror.gyrscale=c["initsgstd"]?vector(c["initsgstd"],1e-6,true):o.imunoise.gyrscale_std;
    o.initstate_std.imuerror.accscale=c["initsastd"]?vector(c["initsastd"],1e-6,true):o.imunoise.accscale_std;
    o.antlever=vector(c["antlever"]);
    return o;
}
static std::vector<std::vector<double>> rows(const std::string &path, size_t columns) {
    std::ifstream in(path);if(!in) throw std::runtime_error("Cannot read "+path);
    std::vector<std::vector<double>> result;std::string line;
    while(std::getline(in,line)) {
        if(line.empty()) continue;
        std::istringstream stream(line);std::vector<double> row;double v;
        while(stream>>v) {if(!std::isfinite(v)) throw std::runtime_error("Nonfinite input");row.push_back(v);}
        if(!stream.eof() || row.size()!=columns || (!result.empty() && row[0]<=result.back()[0]))
            throw std::runtime_error("Invalid input columns or time order: "+path);
        result.push_back(row);
    }
    return result;
}
static AidingObservation observation(const std::vector<double> &r) {
    AidingObservation o;o.time=r[0];o.position=r[1]!=0;o.velocity=r[8]!=0;o.heading=r[15]!=0;
    for(int j=0;j<3;++j) {o.blh[j]=r[2+j];o.position_std[j]=r[5+j];
        o.velocity_ned[j]=r[9+j];o.velocity_std[j]=r[12+j];}
    o.heading_rad=r[16];o.heading_std=r[17];o.baseline_yaw=r[18];
    if((o.position && (o.position_std.array()<=0).any()) ||
       (o.velocity && (o.velocity_std.array()<=0).any()) || (o.heading && o.heading_std<=0))
        throw std::runtime_error("Invalid observation uncertainty");
    return o;
}
int main(int argc,char **argv) {
    try {
        if(argc!=3) throw std::runtime_error("Usage: kf_gins_aided run.yaml aiding.txt");
        const auto c=YAML::LoadFile(argv[1]);auto options=config(c);
        auto im=rows(c["imupath"].as<std::string>(),7), obs=rows(argv[2],19);
        double start=c["starttime"].as<double>(),end=c["endtime"].as<double>();
        if(im.size()<2 || obs.empty() || !std::isfinite(start) || !std::isfinite(end))
            throw std::runtime_error("Insufficient data / invalid interval");
        if(end<0) end=im.back()[0];
        if(start<im.front()[0] || end>604800 || start>=end) throw std::runtime_error("Invalid interval");
        auto engine=std::unique_ptr<GIEngine>(new GIEngine(options));AidingStats stats;
        std::ofstream out(c["outputpath"].as<std::string>()+"/KF_GINS_Navresult.nav");
        if(!out) throw std::runtime_error("Cannot create navigation output");
        out<<std::fixed<<std::setprecision(12);
        // Opt-in offline diagnostics; never added to the board serial protocol.
        std::ofstream diagnostics;
        if(c["diagnostics"] && c["diagnostics"].as<bool>()) {
            diagnostics.open(c["outputpath"].as<std::string>()+"/KF_GINS_State.csv");
            if(!diagnostics) throw std::runtime_error("Cannot create state diagnostics");
            diagnostics<<"gps_tow_s";
            for(const char *group : {"bg_deg_h", "ba_m_s2", "sg_ppm", "sa_ppm"})
                for(const char *axis : {"f", "r", "d"}) diagnostics<<','<<group<<'_'<<axis;
            // Error-state diagonal in native SI units (m, m/s, rad, rad/s,
            // m/s^2, dimensionless); named indices follow the full 21-state core.
            for(int j=0;j<21;++j) diagnostics<<",variance_"<<j;
            diagnostics<<",velocity_nis,heading_nis,velocity_used,velocity_rejected,heading_used,heading_rejected\n";
            diagnostics<<std::setprecision(17);
        }
        size_t i=0,oi=0;while(i<im.size() && im[i][0]<start) ++i;
        while(oi<obs.size() && obs[oi][0]<=start) ++oi;
        const auto readImu=[&im](size_t index) {
            IMU v{};v.time=im[index][0];v.dt=index?im[index][0]-im[index-1][0]:.005;
            if(v.dt<=0 || v.dt>.1) throw std::runtime_error("IMU interval out of range");
            for(int j=0;j<3;++j) {v.dtheta[j]=im[index][1+j];v.dvel[j]=im[index][4+j];}
            return v;
        };
        if(i+1>=im.size()) throw std::runtime_error("No processing interval");
        engine->addImuData(readImu(i),true);
        // Match upstream startup: the first GNSS initializes the state; epochs
        // older than the seed IMU are not applied again as delayed updates.
        while(oi<obs.size() && obs[oi][0]<im[i][0]) ++oi;
        for(++i;i<im.size() && im[i][0]<=end;++i) {
            auto imu=readImu(i);AidingObservation o;const AidingObservation *current=nullptr;
            if(oi<obs.size() && engine->updateBranchFor(imu.time,obs[oi][0])) {
                o=observation(obs[oi++]);current=&o;
                if(oi<obs.size() && engine->updateBranchFor(imu.time,obs[oi][0]))
                    throw std::runtime_error("Multiple aiding epochs in one IMU interval; resample/split input explicitly");
            } else if(oi<obs.size() && obs[oi][0]<engine->timestamp()-.001)
                throw std::runtime_error("Stale aiding epoch");
            engine->addImuData(imu);OfflineAiding::process(*engine,current,stats);
            if(!engine->healthy()) throw std::runtime_error("Nonfinite navigation state / covariance");
            const auto n=engine->getNavState();
            out<<0<<' '<<engine->timestamp()<<' '<<n.pos[0]*R2D<<' '<<n.pos[1]*R2D<<' '<<n.pos[2];
            for(int j=0;j<3;++j) out<<' '<<n.vel[j];
            for(int j=0;j<3;++j) out<<' '<<n.euler[j]*R2D;
            out<<'\n';
            if(diagnostics.is_open()) {
                diagnostics<<engine->timestamp();
                for(int j=0;j<3;++j) diagnostics<<','<<n.imuerror.gyrbias[j]*R2D*3600;
                for(int j=0;j<3;++j) diagnostics<<','<<n.imuerror.accbias[j];
                for(int j=0;j<3;++j) diagnostics<<','<<n.imuerror.gyrscale[j]*1e6;
                for(int j=0;j<3;++j) diagnostics<<','<<n.imuerror.accscale[j]*1e6;
                for(int j=0;j<21;++j) diagnostics<<','<<engine->getCovariance()(j,j);
                diagnostics<<','<<(current && current->velocity ? stats.velocity_nis : -1)
                           <<','<<(current && current->heading ? stats.heading_nis : -1)
                           <<','<<stats.velocity_used<<','<<stats.velocity_rejected
                           <<','<<stats.heading_used<<','<<stats.heading_rejected<<'\n';
            }
        }
        out.close();if(!out) throw std::runtime_error("Navigation write failed");
        if(diagnostics.is_open()) {
            diagnostics.close();if(!diagnostics) throw std::runtime_error("Diagnostics write failed");
        }
        std::cout<<"aiding: velocity_used="<<stats.velocity_used<<" velocity_rejected="<<stats.velocity_rejected
                 <<" heading_used="<<stats.heading_used<<" heading_rejected="<<stats.heading_rejected<<'\n';
        return 0;
    } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}

#include "replay_runner.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <string>
#define CHECK(x) do {if(!(x)) throw std::runtime_error(std::string(__FILE__)+":"+std::to_string(__LINE__)+": " #x);} while(0)
static ReplayRunner runner;
static void reference(const char *folder,unsigned mask) {
    CHECK(runner.begin(mask));NavConfig c;ReplayConfig(c);c.buffer_delay_s=0;
    CHECK(Nav_Initialize(runner.ctx,&c,&replay_imu[0].value)==NAV_OK);
    std::ifstream file(std::string(folder)+"/reference_"+std::to_string(mask)+".nav");CHECK(file.good());
    unsigned gi=0;double max_position=0,max_velocity=0,max_attitude=0;
    for(unsigned i=1;i<replay_imu_count;++i) {
        const auto &im=replay_imu[i].value;
        while(gi<replay_observation_count && replay_observations[gi].value.time_s<=im.time_s+.001) {
            auto o=replay_observations[gi++].value;o.fields &= mask;
            if(o.fields) CHECK(Nav_PushObservation(runner.ctx,&o)==NAV_OK);
        }
        CHECK(Nav_PushImu(runner.ctx,&im)==NAV_OK);uint32_t n=0;
        CHECK(Nav_Process(runner.ctx,im.time_s,1,&n)==NAV_OK && n==1);
        NavOutput result{};CHECK(Nav_GetLatest(runner.ctx,&result));double row[11];
        for(auto &x:row) CHECK(static_cast<bool>(file>>x));
        CHECK(std::abs(result.state_time_s-row[1])<1e-8);
        for(unsigned j=0;j<3;++j) {
            double position=result.state.position[j]-(j<2?row[2+j]*3.14159265358979323846/180:row[2+j]);
            position=std::abs(position)*(j<2?6378137:1);max_position=std::fmax(max_position,position);
            max_velocity=std::fmax(max_velocity,std::abs(result.state.velocity[j]-row[5+j]));
            double attitude=result.state.attitude[j]-row[8+j]*3.14159265358979323846/180;
            max_attitude=std::fmax(max_attitude,std::abs(std::atan2(std::sin(attitude),std::cos(attitude))));
        }
    }
    double extra;CHECK(!(file>>extra));
    CHECK(max_position<1e-4 && max_velocity<1e-6 && max_attitude<1e-7);
    NavStats s{};Nav_GetStats(runner.ctx,&s);CHECK(!s.issue_flags && !s.faulted);
    if(mask==7) CHECK(s.velocity_updates>0 && s.heading_updates>0);
    std::cout<<"mask="<<mask<<" frames="<<s.imu_processed<<" pos_max_m="<<max_position
             <<" velocity_max_mps="<<max_velocity<<" attitude_max_rad="<<max_attitude
             <<" velocity_used="<<s.velocity_updates<<" heading_used="<<s.heading_updates<<'\n';
}
static void timed() {
    CHECK(runner.begin(7));double start=replay_imu[0].value.time_s;
    unsigned step=0;
    while(!runner.done()) {
        double now=start+.001*step++;CHECK(runner.tick(now));
        CHECK(now<replay_imu[replay_imu_count-1].value.time_s+1);
    }
    CHECK(!runner.stats.issue_flags && !runner.stats.late_gnss && !runner.stats.skipped_output_slots);
    CHECK(runner.stats.imu_queue_depth==0 && runner.stats.gnss_queue_depth==0);
    CHECK(runner.max_output_interval_s<.0111);
    double expected=(replay_imu[replay_imu_count-1].value.time_s-replay_imu[1].value.time_s)*100;
    CHECK(std::abs(static_cast<double>(runner.stats.outputs)-expected)<2);
    std::cout<<"recorded-arrival replay: outputs="<<runner.stats.outputs<<" imu_queue_peak="<<runner.stats.imu_queue_peak
             <<" max_age_s="<<runner.stats.max_output_age_s<<" context_bytes="<<Nav_ContextSize()<<'\n';
}
int main(int argc,char **argv) {try {CHECK(argc==2);reference(argv[1],1);reference(argv[1],7);timed();}
catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}}

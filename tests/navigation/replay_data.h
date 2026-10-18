#pragma once
#include "navigation.h"
struct ReplayImu { NavImu value; double arrival_s; };
struct ReplayObservation { NavObservation value; double arrival_s; };
extern const ReplayImu replay_imu[]; // index 0 is the initialization seed
extern const ReplayObservation replay_observations[];
extern const unsigned replay_imu_count, replay_observation_count;
extern const double replay_initial_position[3], replay_initial_velocity[3], replay_initial_attitude[3];
extern const char replay_source_sha256[];
inline void ReplayConfig(NavConfig &c) {
    Nav_DefaultConfig(&c);
    for(unsigned j=0;j<3;++j) {
        c.initial.position[j]=replay_initial_position[j];
        c.initial.velocity[j]=replay_initial_velocity[j];
        c.initial.attitude[j]=replay_initial_attitude[j];
    }
    c.antenna_lever_frd_m[1]=-.15;
}

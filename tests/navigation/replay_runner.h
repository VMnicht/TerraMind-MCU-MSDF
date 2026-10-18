#pragma once
#include "replay_data.h"
// Portable test harness only. The production capture task does not include it.
struct ReplayRunner {
    alignas(8) unsigned char memory[65536];
    NavContext *ctx=nullptr;
    unsigned imu_index=1, observation_index=0, mask=7;
    NavStatus status=NAV_OK;
    NavStats stats{};
    NavOutput output{};
    double first_output_time=0, last_output_time=0, max_output_interval_s=0;
    bool begin(unsigned fields, NavCycleClock clock=nullptr) {
        if(!ctx) ctx=Nav_Construct(memory,sizeof(memory));
        if(!ctx) return false;
        mask=fields;imu_index=1;observation_index=0;stats=NavStats{};output=NavOutput{};
        first_output_time=last_output_time=max_output_interval_s=0;
        NavConfig c;ReplayConfig(c);Nav_SetCycleClock(ctx,clock,nullptr);
        status=Nav_Initialize(ctx,&c,&replay_imu[0].value);return status==NAV_OK;
    }
    bool tick(double now, double (*publication_clock)()=nullptr) {
        if(status!=NAV_OK) return false;
        // Drain both ingress streams before advancing the delayed filter.
        while(imu_index<replay_imu_count && replay_imu[imu_index].arrival_s<=now) {
            status=Nav_PushImu(ctx,&replay_imu[imu_index++].value);
            if(status!=NAV_OK) return false;
        }
        while(observation_index<replay_observation_count && replay_observations[observation_index].arrival_s<=now) {
            auto o=replay_observations[observation_index++].value;o.fields &= mask;
            if(o.fields) {status=Nav_PushObservation(ctx,&o);if(status!=NAV_OK) return false;}
        }
        uint32_t processed=0;status=Nav_Process(ctx,now,8,&processed);
        if(status!=NAV_OK) return false;
        const double publish_now=publication_clock?publication_clock():now;
        if(Nav_PollOutput(ctx,publish_now,&output)) {
            if(!first_output_time) first_output_time=publish_now;
            if(last_output_time && publish_now-last_output_time>max_output_interval_s) max_output_interval_s=publish_now-last_output_time;
            last_output_time=publish_now;
        }
        Nav_GetStats(ctx,&stats);return true;
    }
    bool done() const {return stats.imu_processed==replay_imu_count-1 && observation_index==replay_observation_count;}
};

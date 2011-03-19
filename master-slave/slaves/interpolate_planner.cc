#include "interpolate_planner.h"

    Interpolate_planner::   Interpolate_planner(int slave_num, int statesize, const double* start_state, const double* goal_state, double duration) 
                            : Motion_planner(slave_num, statesize, start_state, goal_state){
        _duration = duration;
    }
    
    Interpolate_planner:: ~Interpolate_planner(){
        //Do nothing
    }
    
    void Interpolate_planner:: get_next_state(const double* cur_state, double* next_state){
        if(is_complete(cur_state)){
            for(int i = 0; i < _statesize; i++){
                next_state[i] = _goal_state[i];
            }
            return;
        }
        else{
            timeval now;
            gettimeofday(&now,NULL);
            get_trajectory_at_time(next_state, now);
        }
    } 
    
    void Interpolate_planner:: get_trajectory_at_time(double* state, timeval state_time){
        double elapsed = timediff(state_time, _time_offset);
        double fractionCompleted = elapsed / _duration;
        for(int i = 0; i < _statesize; i++){
            if(fractionCompleted >= 1)
                state[i] = _goal_state[i];
            else
                state[i] = fractionCompleted * _goal_state[i] + (1 - fractionCompleted) * _start_state[i];
        }
    }
    
    bool Interpolate_planner:: is_complete(const double* state){
        //return false;
        timeval now;
        gettimeofday(&now, NULL);
        double elapsed = timediff(now, _time_offset);
        double fractionCompleted = elapsed / _duration;
        return fractionCompleted > 1.25;
    }
    

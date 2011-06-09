#include "motion_planner.h"

Motion_planner::Motion_planner(int slave_num, int statesize, const double* start_state, const double* goal_state){
        _slave_num = slave_num;
        _statesize = statesize;
        _start_state = new double[statesize];
        copy_state(start_state,_start_state);
        _goal_state = new double[statesize];
        copy_state(goal_state,_goal_state);
        _paused = false;
}

Motion_planner::~Motion_planner(){
    delete[] _start_state;
    delete[] _goal_state;
    delete[] _max_error;
}

void Motion_planner::update_goal(double* new_goal){
    _goal_state = new_goal;
}

bool Motion_planner::is_complete(const double* cur_state){
    /*
    return false; //FIXME
    for(int i = 0; i < _statesize; i++){
        double error = fabs(_goal_state[i] - cur_state[i]);
        if( (i == iX || i == iY || i == iZ) && error > _max_error[i])
            return false;
    }
    return true;
    */
    return false;
}

/* Sets the offset to the current time */
void Motion_planner::begin(){
    gettimeofday(&_time_offset,NULL);
}

void Motion_planner::pause(){
    if(!_paused){
        gettimeofday(&_pause_time,NULL);
        _paused = true;
    }
}

void Motion_planner::unpause(){
    if(_paused){
        timeval now;
        gettimeofday(&now,NULL);
        double pause_dur = timediff(now,_pause_time);
        _time_offset.tv_sec += pause_dur / 1000000;
        _time_offset.tv_usec += pause_dur - (pause_dur / 1000000) * 1000000;
    }
}

void Motion_planner::copy_state(const double* old_state, double* new_state){
    for(int i = 0; i < _statesize; i++){
        new_state[i] = old_state[i];
    }
}
    

#ifndef _MOTION_PLANNER_H_
#define _MOTION_PLANNER_H_

#include "shared.h"
#include "util.h"
class Motion_planner{

    public:
    
    int     _slave_num;
    int     _statesize;
    double* _start_state;
    double* _goal_state;
    double* _max_error;
    timeval _time_offset;
    timeval _pause_time;
    bool _paused;
    
    Motion_planner(int slave_num, int statesize, const double* start_state, const double* goal_state);
    ~Motion_planner();
    
    void update_goal(double* new_goal);
    
    virtual void get_next_state(const double* current_state, double* next_state){};
    virtual void get_trajectory_at_time(double* state, timeval time){};
    virtual bool is_complete(const double* cur_state);
    
    /* Time Operations */
    void begin();
    void pause();
    void unpause();
    
    void copy_state(const double* old_state, double* new_state);
    
   
};

#endif

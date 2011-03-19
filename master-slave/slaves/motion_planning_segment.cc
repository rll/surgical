#include "motion_planning_segment.h"

Motion_planning_segment:: Motion_planning_segment(double duration, Motion_planning_segment* prev_segment){
    _prev_segment = prev_segment;
    _has_absolute_start_pos = false;
    _duration = duration;
}

Motion_planning_segment:: ~Motion_planning_segment(){
    if(_has_absolute_start_pos)
        delete[] _absolute_start_pos;
}

void Motion_planning_segment:: set_absolute_start_pos(double* absolute_start_pos){
    assert(!_has_absolute_start_pos);
    copy_pos(absolute_start_pos,_absolute_start_pos);
    _has_absolute_start_pos = true;
    
}

bool Motion_planning_segment:: get_start(double* start_pos){
    if(_has_absolute_start_pos){
        copy_pos(_absolute_start_pos, start_pos);
        return true;
    }
    else if(_prev_segment && _prev_segment->get_goal(start_pos))
    {
        _has_absolute_start_pos = true;
        copy_pos(start_pos, _absolute_start_pos);
        return true;
    }
    else
        return false;
}


void Motion_planning_segment:: set_grips_to_start(double* pos_to_alter){
    double start[num_dof];
    get_start(start);
    pos_to_alter[iPITCH] = start[iPITCH];
    pos_to_alter[iTILT] = start[iTILT];
    pos_to_alter[iGROSS] = start[iGROSS];
    pos_to_alter[iGRIP] = start[iGRIP];
}


bool Motion_planning_segment:: get_position_at_time(double* position, double time){
     double fractionCompleted = time / _duration;
     double start_pos[num_dof], goal_pos[num_dof];

     bool success = get_start(start_pos) && get_goal(goal_pos);
     if(!success)
        return false;
     else{
        for(int i = 0; i < num_dof; i++){
            if(fractionCompleted >= 1)
               position[i] = goal_pos[i];
            else
                position[i] = fractionCompleted * goal_pos[i] + (1 - fractionCompleted) * start_pos[i];
        }
        return true;
     }
}

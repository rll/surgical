#include "non_visual_segment.h"

Non_visual_segment:: Non_visual_segment(double duration, double* goal_pos, Motion_planning_segment* prev_segment, non_visual_type relative_or_abs)
                    :Motion_planning_segment(duration, prev_segment){

    copy_pos(goal_pos, _goal_pos);                    
    _my_type = relative_or_abs;
}

Non_visual_segment:: ~Non_visual_segment(){
    delete [] _goal_pos;
}

bool Non_visual_segment:: get_goal(double* goal_pos){
    if (_my_type == RELATIVE)
    {
        double start_pos[num_dof];
        if (!get_start(start_pos))
            return false;
        add_pos(_goal_pos, start_pos, goal_pos);
        return true;
    } else {
        copy_pos(_goal_pos, goal_pos);
        return true;
    }
}

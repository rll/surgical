#ifndef _VISUAL_SEGMENT_H_
#define _VISUAL_SEGMENT_H_

#include "motion_planning_segment.h"
#include "visual_feedback.h"

class Visual_segment : public Motion_planning_segment{

public:
    std::vector<double> _servo_times;
    Visual_feedback* _vision;
    bool _requested_goal_pos, _received_goal_pos;
    bool _requested_current_pos, _received_current_pos;
    int _servo_index;
    double _goal_pos[num_dof];
    double _goal_offset_from_servo[num_dof];
    double _goal_offset_from_input[num_dof];

    double _start_pos[num_dof];
    bool _have_start;


    Visual_segment(double duration, std::vector<double>servo_times, Visual_feedback* vision, Motion_planning_segment* prev_segment=NULL, double* goal_offset_input = NULL);
    ~Visual_segment();
    
    /* Either request a goal from visual input, or return the already calculated goal */
    bool get_goal(double* goal_pos);
    //bool get_start(double* start_pos);
    
    /* We get position the same way as any interpolation planner. We also may use this as a "polling" mechanism. In this
     * method, we check whether or not it is time for a new visual servo. */
    bool get_position_at_time(double* position, double time);

    //bool get_position_at_time_with_offsets(double* position, double time);

    /* used for visual servoing - once we get offset, modify the start and end in order to linearly plan to new goal */
    void set_new_start_and_goal();


    /* Checks the given (relative) time and returns true iff we must stop what we're doing and visual servo. */
    bool time_to_servo(double time);
    
    /* Returns true iff a servo request has been sent, but the tip_position has yet to be returned */
    bool currently_servoing();
    
    /* Requests a servo */
    void request_servo();
    

};




#endif

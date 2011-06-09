#ifndef _CONTROLLER_MULTI_PLANNER_H_
#define _CONTROLLER_MULTI_PLANNER_H_

#define NUM_MICROS_MULTI_TO_AVG 350000

#include "accelera_geometry.h"
#include "controller_pid.h"
#include "visual_segment.h"
#include "non_visual_segment.h"



class Controller_multi_planner : public Controller{

public:

    std::vector<Motion_planning_segment*> _segments;
    Controller_pid* _path_follower;
    Accelera_geometry* _geometry;
    double _start_pos[num_dof];
    double _last_target[num_dof];
    double _to_avg_voltages[num_actuators]; bool _avg_voltages;
    int _active_segment_index;
    double _control_clock;
    timeval _last_control;
    int in_port,out_port;
    bool _paused;
    Visual_feedback* _visual_feedback;
    
    Controller_multi_planner(int slave_num, int statesize, int outputsize,double* kp, double* ki, double* kd, Visual_feedback* visual_feedback);
    
    ~Controller_multi_planner();
    
    /* We control by iterating through all segments one by one, and following the given trajectory with our PID controller */
    int     control(const double* state, double* goal, double* out);
    int avg_out_target(const double* state, double* goal, double* out);
    int hold_position(const double* state, double* goal, double* output);
    
    /* Sets the position which the entire planner will start at */
    void set_start_position(const double* start_pos);
    
    /* Appends a new segment which uses visual servoing, with given duration and number of servos */
    void add_visual_segment(double duration, std::vector<double>servo_times = std::vector<double>(), double* goal_offset_input = NULL);
    
    /* Appends a new segment which motion plans to goal_pos in "duration" microseconds */
    void add_non_visual_segment(double duration, double* goal_pos, non_visual_type relative_or_abs = RELATIVE); 


    /* Appends non-visual segment that opens/closes grip */
    void open_grip();
    void close_grip();

    /* Returns the segment whose trajectory is currently being followed */
    Motion_planning_segment* active_segment();
    
    /* Increments to the next active segment, resetting the control clock */
    void go_to_next_segment();
    
    /* Calculates the current "clock time" of our controller */
    void calculate_time();
    
    void init_voltage(const double* voltages);
    void copy_state(const double* original, double* copy);
    void copy_output(const double* original, double* copy);
    
    void get_last_target(double* target);
    bool is_complete();
    
    /* Returns true if we're currently paused */
    bool paused();
    void pause();
    void unpause();
};

#endif



#ifndef _CONTROLLER_VISUAL_SERVO_H_
#define _CONTROLLER_VISUAL_SERVO_H_

#include "controller.h"
#include "controller_lqr_and_pid.h"
#include "motion_planner.h"
#include "interpolate_planner.h"
#include "accelera_geometry.h"
#include "visual_feedback.h"
#include <string.h>

class Controller_visual_servo : public Controller {
    public:
    
    enum Status{
        INIT,
        LQR_CONTROL,
        QUERYING_VISION,
        MOTION_PLANNING,
        RE_QUERYING_VISION,
        ACTING,
        REVERSE_MOTION_PLANNING,
        WAITING,
        COMPLETED
    };
    Status _status;
    Controller_pid* _generic_pid_controller;
    Interpolate_planner* _current_motion_planner;
    Accelera_geometry* _geometry;
    Visual_feedback* _visual_feedback;
    /* For Vision*/
    double _initial_state[num_actuators];
    double _goal_pos[num_actuators];
    double _traj_pos_offsets[num_actuators];
    double _max_error[num_actuators];
    double _duration;
    double _servo_strength[num_actuators];
    double _hold_state[num_actuators];
    int _in_port;
    int _out_port;
    //Time delay till next servo request, in microseconds
    int _servo_frequency;
    timeval _last_servo;
    int _count_servos;
    int _num_servos;
    bool _request_goal;
    double _pos_offset[num_dof];
    
    
    Controller_visual_servo(int slave, int statesize, int outputsize, double* kp, double* ki, double* kd, int num_servos=0, double duration = 5000000, double* pos_offset = NULL, bool request_goal = true);
    ~Controller_visual_servo();
    
    int     control(const double* state, double* goal, double* out);
    void    instantiate_motion_planner_params();
    
    void    copy_state(const double* original, double* copy);
    
    void    query_vision_for_goal();
    void    query_vision_for_update();
    bool    receive_visual_goal();
    bool    receive_visual_update(const double* state);
    bool    needs_new_visual_input();
    void    infer_wrist_positions(double* pos);
    int     hold_position(const double* state, double* out);
    int     act(const double* state, double* out);
    bool    doneActing(const double* state);
    void    servo_adjusted(const double* old_positions, double* new_state);
    void    servo_unadjusted(const double* old_state, double* new_positions);
    void    pause_here(const double* state);
    void    offset_target(double* goal);
    bool    is_complete(const double* state);
    void    get_goal_state(double* goal_state);
};

#endif

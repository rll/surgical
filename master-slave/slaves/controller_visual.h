#ifndef _CONTROLLER_VISUAL_H_
#define _CONTROLLER_VISUAL_H_

#define NUM_SECTIONS 3

#include "controller.h"
#include "controller_lqr_and_pid.h"
#include "controller_visual_servo.h"
#include "controller_motion_planner.h"

#include "accelera_geometry.h"

#include <string.h>

class Controller_visual : public Controller {
    public:
    
    enum Status{
        INIT,
        LQR_CONTROL,
        VISUAL_SERVOING,
        ACTING,
        REVERSE_MOTION_PLANNING,
        WAITING,
        COMPLETED
    };
    Status _status;
    boost::shared_ptr<Controller_visual> _other_slave; //Necessary to tell to stop waiting when I am done my vision loop
    Controller_lqr_and_pid* _standard_controller;
    Controller_visual_servo* _visual_servo_controller;
    Controller_motion_planner* _action_controller;
    Controller_motion_planner* _motion_plan_controller;
    Controller_pid* _generic_pid_controller;
    Visual_feedback* _visual_feedback;
    Time_file*      _target_data;
    Accelera_geometry* _geometry;
    /* For Vision */
    double _initial_state[num_actuators];
    double _return_state[num_actuators];
    double* _kp;
    double* _ki;
    double* _kd;
    bool _logfile_override;
    char _logfile_override_msg[256];
    double _hold_state[num_actuators];
    int _num_servos;
    double _duration;
    double _goal_pos_offset[num_dof];
    const char* _matrixfilepath;
    const char* _targetfilepath;
    double _grip_state[num_actuators];
   
    typedef enum {VISION_FLAG, WAIT_FLAG, NO_FLAG} Flags; 
    
    typedef struct flag_and_time {
        double time;
        Flags flag;
    } flag_and_time;

    Controller_lqr_and_pid* _lqr_controllers[NUM_SECTIONS];
    Flags _flags[NUM_SECTIONS-1];
    int _current_section;
    int _current_flag;
    
    std::list<flag_and_time> _flag_times;
    timeval _lqr_start_time;

    
  
    
    Controller_visual(int slave, int statesize, int outputsize, const char* matrixfilepath, const char* targetfilepath, double* kp, double* ki, double* kd, int num_servos, double duration);
    ~Controller_visual();
    
    int     control(const double* state, double* goal, double* out);
        
    void    copy_state(const double* original, double* copy);
    
    bool    vision_flag();
    bool    wait_flag();
    bool    isVisionFlag(const char* str);
    bool    isWaitFlag(const char* str);

    int     hold_position(const double* state, double* out);
    int    switch_status(Status status, const double* state, double* out);
    void    pause_here(const double* state);
    void    set_other_controller(boost::shared_ptr<Controller> controller);
    void    resume();
    
    bool    ignore_log();
    bool    log_message(char* msg);
    
    void    create_grip_state(const double* state, double* grip_state);
    void    create_ungrip_state(const double* state, double* ungrip_state);
    void    set_goal_pos_offset();
    
    void request_next_goal_position();
    void process_flags();
    
    void slice_chunks();
    
    void get_return_state();
    
    bool is_complete();
    
    Controller_lqr_and_pid* current_lqr_controller();
    Flags next_flag();
};

#endif

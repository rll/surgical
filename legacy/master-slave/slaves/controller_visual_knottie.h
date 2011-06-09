#ifndef _CONTROLLER_VISUAL2_H_
#define _CONTROLLER_VISUAL2_H_

#define NUM_SECTIONS_LEARNED 3
#define NUM_SECTIONS_PLANNED 2
#define NUM_ITERS_LQR_TO_AVG 100

//#include "controller.h"
//#include "controller_lqr_and_pid.h"
#include "controller_multi_planner.h"
#include "controller_lqr_and_pid.h"
#include "controller_pid.h"

#include "accelera_geometry.h"

#include <string.h>

class Controller_visual_knottie : public Controller {
    public:
    
    typedef enum {
        INIT,
        LQR_CONTROL,
        MOTION_PLAN,
        WAITING,
        COMPLETED
    } Status;
    
    Status _status;

    double position[num_dof];
    double _last_target[num_dof];
    double _last_out[num_actuators];
    Accelera_geometry* _geometry;
    
    Controller_multi_planner* _motion_plan_controllers[NUM_SECTIONS_PLANNED];
    Controller_lqr_and_pid* _lqr_controllers[NUM_SECTIONS_LEARNED];
    Controller_pid* _generic_pid_controller;

    boost::shared_ptr<Controller_visual_knottie> _other_slave; //Necessary to tell to stop waiting when I am done my vision loop
  
    Time_file*      _target_data;
    const char* _matrixfilepath;
    const char* _targetfilepath;
    //double _first_state_next_lqr[num_actuators];

    bool _logfile_override;
    char _logfile_override_msg[256];
    //double _hold_state[num_actuators];
   
    typedef enum {VISION_FLAG, WAIT_FLAG, NO_FLAG} Flags; 

    Flags _flags[NUM_SECTIONS_PLANNED];
    int _current_lqr_section;
    int _current_flag;
    int _current_motion_plan_section;
    
    timeval _lqr_start_time;
    Visual_feedback* _visual_feedback;
    int _num_iters_since_lqr_start;

  

    Controller_visual_knottie(int slave_num_, int statesize, int outputsize, const char* matrixfilepath, const char* targetfilepath, double* kp, double* ki, double* kd);
    ~Controller_visual_knottie();
    
    int control(const double* state, double* goal, double* out);
       

    int  hold_position(const double* state, const double* state_to_hold, double* out);
    int switch_status(Status status, const double* state, double* out);

    void copy_state(const double* original, double* copy);
    void copy_output(const double* original, double* copy);
    void zero_state(double* statecopy);
   

    Flags next_flag();
    bool vision_flag();
    bool wait_flag();
    bool isVisionFlag(const char* str);
    bool isWaitFlag(const char* str);
   
    void    set_other_controller(boost::shared_ptr<Controller> controller);
    void    resume();
    
    bool    ignore_log();
    bool    log_message(char* msg);
    
    void request_next_goal_position();
    void process_flags();
    
    void get_first_state_lqr_file(int lqr_file_num, double* data);
    void get_last_state_lqr_file(int lqr_file_num, double* data);
    bool is_complete();
    
    Controller_lqr_and_pid* current_lqr_controller();
    Controller_multi_planner* current_motion_plan_controller();

    void add_pickup(Controller_multi_planner* motion_plan_controller);


};

#endif

#include "controller_visual.h"
#include <cstdio>

Controller_visual:: Controller_visual(int slave, int statesize, int outputsize, const char* matrixfilepath, const char* targetfilepath, double* kp, double* ki, double* kd, int num_servos, double duration)
                    : Controller(slave, statesize, outputsize){

    _status = INIT;
    _standard_controller = new Controller_lqr_and_pid(slave, statesize, outputsize, matrixfilepath, targetfilepath, kp, ki, kd);
    _motion_plan_controller = NULL;
    _visual_servo_controller = NULL;
    _action_controller = NULL;
    _matrixfilepath = matrixfilepath;
    _targetfilepath = targetfilepath;
    _generic_pid_controller = new Controller_pid(slave, statesize, outputsize, kp, ki, kd); //Adding I term
    _target_data = new Time_file(targetfilepath, TRUE_STATE_SIZE); //Need another ifstream to check and see if there is a flag.
    
    _geometry = new Accelera_geometry(slave);
    //_other_slave = NULL;
    _logfile_override = false;
    sprintf(_logfile_override_msg, "NOT A VALID MESSAGE\n");
    _num_servos = num_servos;
    _duration = duration;
    
    /* Added for request override only */
    int in_port,out_port;
    if (slave == 1)
    {
        in_port = CONTROLLER_IN_PORT_SLAVE1;
        out_port = VISION_IN_PORT_SLAVE1;
    } else {
        in_port = CONTROLLER_IN_PORT_SLAVE2;
        out_port = VISION_IN_PORT_SLAVE2;
    }
    _visual_feedback = new Visual_feedback(in_port, out_port, VISION_IP);
    /* End of request override additions */
    
    /* Initialize vectors for vision and wait flags */
    _flag_times = std::list<flag_and_time>();
    process_flags();
    
    /* Added to make LQR easier */
    char sectionmatrixfile[256];
    char sectiontargetfile[256];
    for(int i = 0; i < NUM_SECTIONS; i++){
        sprintf(sectionmatrixfile,"%s%d",matrixfilepath,i+1);
        sprintf(sectiontargetfile,"%s%d",targetfilepath,i+1);    
        _lqr_controllers[i] = new Controller_lqr_and_pid(slave, statesize, outputsize, sectionmatrixfile, sectiontargetfile, kp, ki, kd);
    }
    if(slave == 1){
        _flags[0] = VISION_FLAG;
        _flags[1] = WAIT_FLAG;
    }
    else{
        _flags[0] = WAIT_FLAG;
        _flags[1] = VISION_FLAG;
    }
    _current_section = -1;
    _current_flag = -1;
    /* End of addition */
    
    
    _kp = new double[statesize];
    _ki = new double[statesize];
    _kd = new double[statesize];
    copy_state(kp,_kp);
    copy_state(ki,_ki);
    copy_state(kd,_kd);
}
    
Controller_visual:: ~Controller_visual(){
    if(_standard_controller) delete _standard_controller;
    if(_visual_servo_controller) delete _visual_servo_controller;
    if(_motion_plan_controller) delete _motion_plan_controller;
    if(_generic_pid_controller) delete _generic_pid_controller;
    _target_data->close();
    delete _geometry;
    delete[] _lqr_controllers;
}

/** Our control follows a set flow:
 * -> LQR_CONTROL: Default controller, untill we see a vision flag. In which case, we...
 * -> VISUAL_SERVOING: Running visual servo to goal.
 * -> ACTING: At our destination, we will perform some action. In this case, that action is gripping.
 * -> REVERSE_MOTION_PLANNING: Motion plan back to our original position (or, as the case may be, a new final position) */
int Controller_visual:: control(const double* state, double* goal, double* out){
    printf("Currently at state: %f, %f, %f\n",state[mBASE_LEFT],state[mBASE_REAR],state[mBASE_RIGHT]);
    switch(_status){
        case INIT:
            return switch_status(LQR_CONTROL, state, out);
        break;
        case LQR_CONTROL:
            if(vision_flag()){
                _visual_servo_controller = new Controller_visual_servo(slave_num, state_size, output_size, _kp, _ki, _kd, _num_servos, _duration, _goal_pos_offset); 
                _logfile_override = true;
                sprintf(_logfile_override_msg, "%%V%%\n");
                //return switch_status(VISUAL_SERVOING, state, out);
                printf("Leaving LQR at state: %f, %f, %f\n",state[mBASE_LEFT],state[mBASE_REAR],state[mBASE_RIGHT]);
                return switch_status(VISUAL_SERVOING, state, out);
                //return switch_status(WAITING, state, out);
                
            }
            else if(wait_flag()){
               _logfile_override = true;
               sprintf(_logfile_override_msg, "%%W%%\n");
               return switch_status(WAITING, state, out);
            }
            
            return _standard_controller->control(state, goal, out);

        break;
         
        case VISUAL_SERVOING:
            if(_visual_servo_controller->is_complete(state)){
                //delete _visual_servo_controller;
                _visual_servo_controller = NULL;
                hold_position(state,out);
                double ungrip_state[num_actuators];
                create_ungrip_state(_hold_state, ungrip_state);
                double grip_state[num_actuators];
                create_grip_state(_hold_state, grip_state);
                _action_controller = new Controller_motion_planner(slave_num, state_size, output_size, _kp, _ki, _kd,  ungrip_state, grip_state, 1500000); 
                return switch_status(ACTING, state, out);
            }            
            return _visual_servo_controller->control(state, goal, out);    
        break;
            
        case ACTING:
            if(_action_controller->is_complete(state)){
                _motion_plan_controller = new Controller_motion_planner(slave_num, state_size, output_size, _kp, _ki, _kd, state, _return_state, 5000000);
                return switch_status(REVERSE_MOTION_PLANNING, state, out);
            }
            return _action_controller->control(state, goal, out);   
         break;
            
        case REVERSE_MOTION_PLANNING:
            if(_motion_plan_controller->is_complete(state)){
                //delete _motion_plan_controller;
                _motion_plan_controller = NULL;
                if(_other_slave){
                    _other_slave->resume();
                }
                /*
                 _standard_controller = new Controller_lqr_and_pid(slave_num, state_size, output_size, _matrixfilepath, _targetfilepath, _kp, _ki, _kd);
                 _standard_controller->base_controller->target_data
                 */
                return switch_status(LQR_CONTROL, state, out);
            }
            return _motion_plan_controller->control(state, goal, out);  
            break;
            
        case WAITING:
            return hold_position(state, out);     
            break;
                   
    }
}


void Controller_visual:: copy_state(const double* original, double* copy){
    for(int i = 0; i < state_size; i++){
        copy[i] = original[i];   
    }
}/** Reads the current target file and notifies in the event of a vision flag */
bool Controller_visual:: vision_flag(){
    _target_data->seekg(_standard_controller->base_controller->target_data->tellg());
    //std::cout << _standard_controller->base_controller->target_data->tellg();
    //std::cout << _standard_controller->base_controller->target_data->tellg() << std::endl;
    char str[20];
    _target_data->getline(str, 20);
    timeval now;
    gettimeofday(&now, NULL);
    if(!_flag_times.empty() && _flag_times.front().flag == VISION_FLAG && timediff(now, _lqr_start_time) >= _flag_times.front().time){
        _flag_times.pop_front();
       
        /* Keep track of next to-> position */
        _target_data->reset_time();
        std::vector<double> data = _standard_controller->base_controller->target_data->data();
        for(int i = 0; i < num_actuators; i++){
            _return_state[i] = data[i];
        }
        _standard_controller->base_controller->matrix_data->data(); //Just to empty and make sure we're at 0
        return true;
    }
    return false;
}

/** Reads the current target file and notifies in the event of a wait flag */
bool Controller_visual:: wait_flag(){
    _target_data->seekg(_standard_controller->base_controller->target_data->tellg());
    char str[256];
    _target_data->getline(str, 256);
    timeval now;
    gettimeofday(&now, NULL);
    if(!_flag_times.empty() && _flag_times.front().flag == WAIT_FLAG && timediff(now, _lqr_start_time) >= _flag_times.front().time){
        _flag_times.pop_front();
        _standard_controller->base_controller->matrix_data->data(); //Just to empty and make sure we're at 0
        _standard_controller->base_controller->target_data->data(); //Just to empty and make sure we're at 0
        return true;
    }
    return false;
}



/** Checks if given string is a vision flag */
bool Controller_visual:: isVisionFlag(const char* str){
    if(str[0] == '%' && str[1] == 'V'){
        return true;
    }
    else
        return false;
}

/** Checks if given string is a vision flag */
bool Controller_visual:: isWaitFlag(const char* str){
    if(str[0] == '%' && str[1] == 'W')
        return true;
    else
        return false;
}


int Controller_visual:: switch_status(Status status, const double* state, double* out){
    _status = status;
    pause_here(state);
    /* Added to temporarily let us request the knot position early. Remove after!
    if(status == LQR_CONTROL)
        _visual_feedback->request_goal_position();
     End of temporary additions */
     if(status == LQR_CONTROL){
        gettimeofday(&_lqr_start_time,NULL);
        _standard_controller->reset_time();
     }
    return hold_position(state, out);
}

/** Holds position, generally as a failsafe measure while waiting on some other input */ 
int  Controller_visual:: hold_position(const double* state, double* out){
    double goal[state_size];
    copy_state(_hold_state,goal);
    //printf("Goal at: %f, %f, %f\n",goal[mBASE_LEFT],goal[mBASE_REAR],goal[mBASE_RIGHT]);
    return _generic_pid_controller->control(state, goal, out);   
}

void    Controller_visual:: pause_here(const double* state){
    copy_state(state, _hold_state);
    printf("Pausing at: %f, %f, %f\n",_hold_state[mBASE_LEFT],_hold_state[mBASE_REAR],_hold_state[mBASE_RIGHT]);
}

/** Gives me a pointer to another slave controller, which I am responsible for making wait and resume the moment I see a flag */
void Controller_visual:: set_other_controller(boost::shared_ptr<Controller> controller){
    _other_slave = boost::shared_dynamic_cast<Controller_visual>(controller);
}

/** Called by another slave controller, to tell me to stop waiting once it has done its job*/
void Controller_visual:: resume(){
    double out[num_actuators]; //Won't actually be used for anything, just to appease the switch_status gods.
    switch_status(LQR_CONTROL,_hold_state,out);
}

/* If we are running anything but LQR control, don't log! */
bool Controller_visual:: ignore_log(){
    return _status != LQR_CONTROL;
}

bool Controller_visual:: log_message(char* msg){
    if(_logfile_override){
        strcpy(msg,_logfile_override_msg);
        _logfile_override = false;
    }
}

/* Sets the "grip" to be at the current state, minus the offset, plus grip */
void Controller_visual:: create_grip_state(const double* state, double* grip_state){
    double position[num_dof];
    double grip_position[num_dof];
    _geometry->motors_to_point(state, position);
    for(int i = 0; i < num_dof; i++){
        grip_position[i] = position[i] - _goal_pos_offset[i];
    }
    grip_position[iGRIP] = GRIP_ON;
    _geometry->point_to_motors(grip_position, grip_state);
}

/* Sets an ungrip state at the current state */
void Controller_visual:: create_ungrip_state(const double* state, double* ungrip_state){
    double position[num_dof];
    double ungrip_position[num_dof];
    _geometry->motors_to_point(state, position);
    for(int i = 0; i < num_dof; i++){
        ungrip_position[i] = position[i];
    }
    ungrip_position[iGRIP] = GRIP_ON;
    _geometry->point_to_motors(ungrip_position, ungrip_state);
}

void Controller_visual:: set_goal_pos_offset(){
    for(int i = 0; i < num_dof; i++){
        _goal_pos_offset[i] = 0;
    }
    _goal_pos_offset[iX] = 0.0;
    _goal_pos_offset[iY] = 0.0;
    _goal_pos_offset[iZ] = 15.0;
}

void Controller_visual:: request_next_goal_position(){
    _visual_feedback->request_goal_position();
}

void Controller_visual:: process_flags(){
    char token[1024];
    double timeprev = -1;
    do{
        _target_data->getline(token, 1024);
        std::stringstream ss(token);
        
        if(isVisionFlag(token)) {
            flag_and_time temp;
            temp.time = timeprev-0.01 - 1000000;
            temp.flag = VISION_FLAG;
            _flag_times.push_back(temp);
        }    
        else if(isWaitFlag(token)) {
            flag_and_time temp;
            temp.time = timeprev+0.01 - 1000000;
            temp.flag = WAIT_FLAG;
            _flag_times.push_back(temp);
        } else {
            ss >> timeprev;
        }
        
    } while (token[0] != 0);  
}


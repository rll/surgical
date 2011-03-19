#include "controller_visual_servo.h"
Controller_visual_servo:: Controller_visual_servo(int slave, int statesize, int outputsize, double* kp, double* ki, double* kd, int num_servos, double duration, double* pos_offset, bool request_goal)
                    : Controller(slave, statesize, outputsize){

    _status = INIT;
    ki[mBASE_RIGHT] = (kp[mBASE_RIGHT]/500.0);
    ki[mBASE_LEFT] = (kp[mBASE_LEFT]/500.0);
    ki[mBASE_REAR] = (kp[mBASE_REAR] /500.0);
    _generic_pid_controller = new Controller_pid(slave, statesize, outputsize, kp, ki, kd); //Adding I term
    _current_motion_planner = NULL;
    _geometry = new Accelera_geometry(slave);
    if (slave == 1)
    {
        _in_port = CONTROLLER_IN_PORT_SLAVE1;
        _out_port = VISION_IN_PORT_SLAVE1;
    } else {
        _in_port = CONTROLLER_IN_PORT_SLAVE2;
        _out_port = VISION_IN_PORT_SLAVE2;
    }
    _visual_feedback = new Visual_feedback(_in_port, _out_port, VISION_IP);
    _duration = duration;
    instantiate_motion_planner_params();
    _count_servos = 0;
    _num_servos = num_servos;
    
    if(pos_offset){
        for(int i = 0; i < num_dof; i++){
            _pos_offset[i] = pos_offset[i];
        }
    }
    else{
        for(int i = 0; i < num_dof; i++){
            _pos_offset[i] = 0;
        }
    }
    _request_goal = request_goal;
}
    
Controller_visual_servo:: ~Controller_visual_servo(){
    if(_current_motion_planner) delete _current_motion_planner;
    if(_generic_pid_controller) delete _generic_pid_controller;
    if(_visual_feedback) delete _visual_feedback;
    delete _geometry;
    delete[] _initial_state;
    delete[] _goal_pos;
    delete[] _hold_state;
    delete[] _servo_strength;
}

/** Our control follows a set flow:
 * -> QUERYING_VISION: Ask vision for our goal. Til we get it, we wait.
 * -> MOTION_PLANNING: Motion plan to the goal, as given by the vision machine. That is, until we decide we need feedback.
 * -> RE_QUERYING_VISION: Ask vision where it thinks we are. We then update our goal accordingly, and continue motion planning in this loop.
 * -> ACTING: At our destination, we will perform some action. In this case, that action is gripping. */
int Controller_visual_servo:: control(const double* state, double* goal, double* out){
    
    switch(_status){
         case INIT:
            //printf("INITIALIZING...\n");
            query_vision_for_goal();
            _status = QUERYING_VISION;
            pause_here(state);
            return hold_position(state, out);   
         break;
         
         case QUERYING_VISION:
            if(!_visual_feedback->received()){        
                return hold_position(state, out);
            }
            else if(receive_visual_goal()){
                double init_position[num_dof];
                printf("RECEIVED THREAD LOCATION.\n");
                _geometry->motors_to_point(state, init_position);
                _current_motion_planner = new Interpolate_planner(slave_num, num_dof, init_position, _goal_pos, _duration);
                _current_motion_planner->begin();
                copy_state(state, _initial_state);

                _status = MOTION_PLANNING;
                return hold_position(state, out);
            }
            return hold_position(state,out);
         break;
         
         case MOTION_PLANNING:
            double position[num_dof];
            _geometry->motors_to_point(state,position);
            //printf("Currently at Position: %f, %f, %f\n",position[iX],position[iY],position[iZ]);
            double planning_state_pos[num_dof];
            //servo_unadjusted(state,planning_state_pos);
            _geometry->motors_to_point(state, planning_state_pos);
            //printf("Adjusted for offset of 0\n");
            if(_current_motion_planner->is_complete(planning_state_pos)){
                //delete _current_motion_planner;
                _current_motion_planner = NULL;
                _status = COMPLETED;
                double goal_state[num_actuators];
                get_goal_state(goal_state);
                //printf("Visual Servo Is Complete!\n");
                pause_here(goal_state);
                return hold_position(state, out);
            }
            else if(needs_new_visual_input()){
                //printf("NEEDS VISUAL UPDATE\n");
                query_vision_for_update();
                _current_motion_planner->pause();
                _status = RE_QUERYING_VISION;
                printf("Re Querying Vision!\n");
                _count_servos++;
                double next_state[state_size];
                double next_planning_state_pos[num_dof];
                _current_motion_planner->get_next_state(planning_state_pos, next_planning_state_pos);
                _geometry->point_to_motors(next_planning_state_pos,next_state);
                pause_here(next_state);
                return hold_position(state, out);
            }
            else{
                //printf("Should move.\n");
                double next_state[state_size];
                double next_planning_state_pos[state_size];
                _current_motion_planner->get_next_state(planning_state_pos, next_planning_state_pos);
                //servo_adjusted(next_planning_state_pos,next_state); //IMPORTANT!
                _geometry->point_to_motors(next_planning_state_pos,next_state);
                printf("Motion Planning To: %f, %f, %f\n", next_planning_state_pos[iX], next_planning_state_pos[iY], next_planning_state_pos[iZ]);
                printf("With Final Goal Position: %f, %f, %f\n",_goal_pos[iX],_goal_pos[iY],_goal_pos[iZ]);
                printf("With Offset: %f, %f, %f\n",_traj_pos_offsets[iX],_traj_pos_offsets[iY],_traj_pos_offsets[iZ]);
                double goal_motors[state_size];
                _geometry->point_to_motors(_goal_pos, goal_motors, false);
                printf("With Final Motor Position: %f, %f, %f\n", goal_motors[mBASE_LEFT], goal_motors[mBASE_REAR], goal_motors[mBASE_RIGHT]);
                printf("With Current Motor Position: %f, %f, %f\n", state[mBASE_LEFT], state[mBASE_REAR], state[mBASE_RIGHT]);
                return _generic_pid_controller->control(state, next_state, out);
                
            }
         break;
            
        case RE_QUERYING_VISION:
            //printf("RE-QUERYING VISION\n");
            if(receive_visual_update(state)){
                printf("RECEIVED VISUAL UPDATE!\n");
                _current_motion_planner->unpause();
                _status = MOTION_PLANNING;
                //_status = WAITING;
                return hold_position(state, out);
            }
            return hold_position(state,out);
         break;
         
         case WAITING:
            //printf("WAITING.\n");
            return hold_position(state,out);
           break;
           
         case COMPLETED:
            //printf("SERVOING COMPLETE.\n");
            return hold_position(state,out);
         break;
    }
    
    return 1;  
}

void Controller_visual_servo:: instantiate_motion_planner_params(){

    _max_error[iPITCH] = 0.2;
    _max_error[iGRIP] = 0.2;
    _max_error[iTILT] = 0.2;
    _max_error[iGROSS] = 0.2;
    _max_error[iX] = 0.5;
    _max_error[iY] = 0.5;
    _max_error[iZ] = 0.5;


    _servo_strength[iPITCH] = 0.9;
    _servo_strength[iGRIP] = 0.9;
    _servo_strength[iTILT] = 0.9;
    _servo_strength[iGROSS] = 0.9;
    _servo_strength[iX] = 1.0; //changed from 0.9
    _servo_strength[iY] = 1.0;
    _servo_strength[iZ] = 1.0;
    
    for(int i = 0; i < num_dof; i++){
        _traj_pos_offsets[i] = 0;
    }
    
    //Request new servo every 2.5 seconds
    _servo_frequency = 4.0/5.0 * _duration ;
}

void Controller_visual_servo:: copy_state(const double* original, double* copy){
    for(int i = 0; i < state_size; i++){
        copy[i] = original[i];   
    }
}

/** Queries the vision computer to show us the goal (i.e. the thread position).*/
void Controller_visual_servo:: query_vision_for_goal(){
    if(_request_goal)
        _visual_feedback->request_goal_position();
}

/** Queries the vision computer to tell us where the tip of the endaffector currently is. */
void Controller_visual_servo:: query_vision_for_update(){
    _visual_feedback->request_current_position();
}

/** Checks to see if we've received the goal state from the vision computer. If we have, set _current_goal accordingly */
bool Controller_visual_servo:: receive_visual_goal(){
    double pos[num_dof];
    if(_visual_feedback->get_goal_position(_goal_pos)){
        printf("Before inferring wrist positions: %f, %f, %f\n",_goal_pos[iX],_goal_pos[iY],_goal_pos[iZ]);
        infer_wrist_positions(_goal_pos);
        printf("Before offsetting the target: %f, %f, %f\n",_goal_pos[iX],_goal_pos[iY],_goal_pos[iZ]);
        printf("With additional offset: %f, %f, %f\n",_pos_offset[iX],_pos_offset[iY],_pos_offset[iZ]);
        offset_target(_goal_pos);
        printf("Aiming for positions: %f, %f, %f\n",_goal_pos[iX],_goal_pos[iY],_goal_pos[iZ]);
        gettimeofday(&_last_servo, NULL);
        return true;
    }
    else
        return false;
}

/** Checks to see if we've received the visual estimation of where the tip of our endaffector is. If we have,
  * alters _current_state to compensate for the error between state and vision's estimate */
bool Controller_visual_servo:: receive_visual_update(const double* state){
    double camera_pos[num_dof];
    if(_visual_feedback->get_current_position(camera_pos)){
        infer_wrist_positions(camera_pos);
        double true_pos[num_dof];
        _geometry->motors_to_point(state, true_pos);
        double intended_pos[num_dof];
        //servo_unadjusted(state, true_pos);
        _geometry->motors_to_point(_hold_state, intended_pos);
        double delta_traj[num_dof];
        for(int i = 0; i < num_dof; i++){
            delta_traj[i] = _servo_strength[i] * (intended_pos[i] - camera_pos[i]);
            _goal_pos[i] += delta_traj[i] - _traj_pos_offsets[i];
            _traj_pos_offsets[i] =  delta_traj[i];// FIXME?
            
            printf("Camera Pos: %f - Intended_Pos: %f = %f\n",camera_pos[i],intended_pos[i],camera_pos[i] - intended_pos[i]);
        }
        //delete _current_motion_planner;
        _current_motion_planner = new Interpolate_planner(slave_num, num_dof, intended_pos, _goal_pos, _duration);
        _current_motion_planner->begin();
        gettimeofday(&_last_servo, NULL);
        return true;
    }
    else
        return false;
}

/** Checks whether or not it is time to update once more (visual servoing) */
bool Controller_visual_servo:: needs_new_visual_input(){
    if(_count_servos >= _num_servos)
        return false;
    timeval now;
    gettimeofday(&now, NULL);
    double diff = timediff(now, _last_servo);
    return diff >= _servo_frequency;
}

/** Performs an action. In this case, that action is gripping. NOTE: May want that action to be LQR learned, or PID learned. */
int  Controller_visual_servo:: act(const double* state, double* out){
    double pos[num_dof];
    _geometry->motors_to_point(state, pos, false);
    pos[iGRIP] = GRIP_ON;
    double grip_state[num_dof];
    _geometry->point_to_motors(pos, grip_state, false);
    return _generic_pid_controller->control(state, grip_state, out);
    
}

/** Confirms whether or not said action has been completed */
bool Controller_visual_servo:: doneActing(const double* state){
    double pos[num_dof];
    _geometry->motors_to_point(state, pos, false);
    return pos[iGRIP] >= 1;
}

/** Holds position, generally as a failsafe measure while waiting on some other input */ 
int  Controller_visual_servo:: hold_position(const double* state, double* out){
    double goal[state_size];
    copy_state(_hold_state,goal);
    //printf("Holding position at: %f, %f, %f\n",goal[mBASE_LEFT],goal[mBASE_REAR],goal[mBASE_RIGHT]);
    return _generic_pid_controller->control(state, goal, out);   
}

/** Infers the wrist positions from the goal base positions, and populates pos accordingly */
void Controller_visual_servo:: infer_wrist_positions(double* pos){
    //FIXME Currently assigns rather arbitrary wrist positions.
    pos[iPITCH] = M_PI/2;
    pos[iTILT] = M_PI/2;
    pos[iGROSS] = M_PI;
    pos[iGRIP] = GRIP_OFF;
}

void Controller_visual_servo:: servo_adjusted(const double* old_positions, double* new_state){
    double new_positions[num_dof];
    for(int i = 0; i < num_dof; i++){
        new_positions[i] = old_positions[i] + _traj_pos_offsets[i];
    }
    _geometry->point_to_motors(new_positions, new_state, false);
}

void Controller_visual_servo:: servo_unadjusted(const double* old_state, double* new_positions){
    
    double old_positions[num_dof];
    _geometry->motors_to_point(old_state, old_positions, false);
    for(int i = 0; i < num_dof; i++){
        new_positions[i] = old_positions[i] - _traj_pos_offsets[i];
    }
    
}

void    Controller_visual_servo:: pause_here(const double* state){
    copy_state(state, _hold_state);
    printf("Pausing at: %f, %f, %f\n",_hold_state[mBASE_LEFT],_hold_state[mBASE_REAR],_hold_state[mBASE_RIGHT]);
}

void    Controller_visual_servo:: offset_target(double* goal){
    for(int i = 0; i < num_dof; i++){
        goal[i] += _pos_offset[i];
    }
}

bool Controller_visual_servo:: is_complete(const double* state){
    return _status == COMPLETED;
}

void Controller_visual_servo:: get_goal_state(double* goal_state){
    _geometry->point_to_motors(_goal_pos,goal_state);
}


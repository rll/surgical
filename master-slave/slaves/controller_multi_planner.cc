#include "controller_multi_planner.h"

Controller_multi_planner::  Controller_multi_planner(int slave_num_, int statesize, int outputsize,double* kp, double* ki, double* kd, Visual_feedback* visual_feedback)
                        :   Controller(slave_num_, statesize, outputsize){

    _segments = std::vector<Motion_planning_segment*>();         
    _path_follower = new Controller_pid(slave_num_, statesize, outputsize, kp, ki, kd);
    _geometry = new Accelera_geometry(slave_num_);
    _control_clock = 0;
    
    _visual_feedback = visual_feedback;

    _paused = false;
    _active_segment_index = 0;
    _avg_voltages = false;
}

Controller_multi_planner:: ~Controller_multi_planner(){
    for(int i = 0; i < _segments.size(); i++)
        delete _segments.at(i);
        
    delete &_segments;
    delete _path_follower;
}

//goal is unused?
int Controller_multi_planner:: control(const double* state, double* goal, double* output){
    double position[num_dof];
     _geometry->motors_to_point(state,position);
    
    calculate_time();
    if(_control_clock > active_segment()->_duration){
        go_to_next_segment();
        calculate_time(); //We call calculate_time() again to ensure that we perfectly sync up for this new segment.
        if (is_complete())
            return -1;
    }
    //printf("Trying segment number  %d   total size: %d \n", _active_segment_index+1, _segments.size());
    double target[num_dof];
    bool moving = active_segment()->get_position_at_time(target, _control_clock);

    double curr_goal[num_dof];
    active_segment()->get_goal(curr_goal);
    //printf("Current Goal: %f, %f, %f, %f, %f, %f, %f\n", curr_goal[iX], curr_goal[iY], curr_goal[iZ], curr_goal[iPITCH], curr_goal[iTILT], curr_goal[iGROSS], curr_goal[iGRIP]);

    double target_motors[state_size];
    if(moving){
        copy_pos(target, _last_target);
        if(paused())
            unpause();
        return avg_out_target(state, goal, output);
    }
    else{
        if(!paused())
            pause();
        return hold_position(state, target_motors, output);
   
   }
}


int Controller_multi_planner:: hold_position(const double* state, double* goal, double* output)
{
    double target_motors[state_size];
    _geometry->point_to_motors(_last_target, target_motors);    
    return _path_follower->control(state, target_motors, output);
}

int Controller_multi_planner::avg_out_target(const double* state, double* goal, double* out)
{
   double target_motors[state_size];
   _geometry->point_to_motors(_last_target, target_motors);    
   int to_return = _path_follower->control(state, target_motors, out);
   if (_avg_voltages)
   {
        if (_control_clock < NUM_MICROS_MULTI_TO_AVG)
        {
            for (int i=0; i < num_actuators; i++)
            {
                double frac = _control_clock/NUM_MICROS_MULTI_TO_AVG;
                out[i] = frac*out[i] + (1-frac)*_to_avg_voltages[i];
            }

        } else{
            _avg_voltages= false;
        }

   }

   return to_return;

}



void Controller_multi_planner:: set_start_position(const double* start_pos){
    //_geometry->motors_to_point(start_pos, _start_pos);
    copy_pos(start_pos, _start_pos);
/*    _start_pos[iGRIP] = GRIP_OFF;
    _start_pos[iPITCH] = M_PI/2;
    _start_pos[iTILT] = M_PI/2;
    _start_pos[iGROSS] = M_PI;*/
    copy_pos(_start_pos, _last_target); // ensures _last_target will, at worst, initialize to our starting position
    printf("Start: %f, %f, %f, %f, %f, %f, %f\n", _last_target[iX], _last_target[iY], _last_target[iZ], _last_target[iPITCH], _last_target[iTILT], _last_target[iGROSS], _last_target[iGRIP]);
    _active_segment_index = 0;
}


void Controller_multi_planner:: add_visual_segment(double duration, std::vector<double>servo_times, double* goal_offset_input){
    Visual_segment* segment;
    if(_segments.empty()){
        segment = new Visual_segment(duration, servo_times, _visual_feedback, NULL, goal_offset_input);
        segment->set_absolute_start_pos(_start_pos);
    }
    else{
        segment = new Visual_segment(duration, servo_times, _visual_feedback, _segments.back(), goal_offset_input);
    }
    _segments.push_back(segment);
}
    

void Controller_multi_planner:: add_non_visual_segment(double duration, double* goal_pos, non_visual_type relative_or_abs){
    Non_visual_segment* segment;
    if(_segments.empty()){
        segment = new Non_visual_segment(duration, goal_pos, NULL, relative_or_abs);
        segment->set_absolute_start_pos(_start_pos);
    }
    else{
        segment = new Non_visual_segment(duration, goal_pos, _segments.back(), relative_or_abs);
    }
    _segments.push_back(segment);
}


void Controller_multi_planner::open_grip(){
    double alter_grip[num_dof];
    for (int i = 0; i < num_dof; i++)
        alter_grip[i] = 0.0;

    alter_grip[iGRIP] = -1.0*GRIP_ON;
    add_non_visual_segment(2, alter_grip);
}

void Controller_multi_planner::close_grip(){
    double alter_grip[num_dof];
    for (int i = 0; i < num_dof; i++)
        alter_grip[i] = 0.0;

    alter_grip[iGRIP] = 1.0*GRIP_ON;
    add_non_visual_segment(2, alter_grip);
}



Motion_planning_segment* Controller_multi_planner:: active_segment(){
    return _segments.at(_active_segment_index);
}

void Controller_multi_planner:: go_to_next_segment(){
    _active_segment_index++;
    _control_clock = 0.0;
}

void Controller_multi_planner:: calculate_time(){
    if(_control_clock == 0.0)
        gettimeofday(&_last_control, NULL); //Don't want an initial jump.
    if(!paused()){
        timeval now;
        gettimeofday(&now, NULL);
        _control_clock += timediff(now,_last_control);
    }
    gettimeofday(&_last_control, NULL);
}


void Controller_multi_planner::get_last_target(double* target)
{
    copy_pos(_last_target, target);
}


bool Controller_multi_planner::is_complete(){
    return  _active_segment_index >= _segments.size();
}

void Controller_multi_planner:: init_voltage(const double* voltages)
{
    copy_output(voltages, _to_avg_voltages);       
    _avg_voltages = true;
}

void Controller_multi_planner:: copy_state(const double* original, double* copy){
    for(int i = 0; i < state_size; i++){
        copy[i] = original[i];   
    }
}

void Controller_multi_planner:: copy_output(const double* original, double* copy){
    for(int i = 0; i < output_size; i++){
        copy[i] = original[i];   
    }
}

bool Controller_multi_planner:: paused(){
    return _paused;
}

void Controller_multi_planner:: pause(){
    _paused = true;
}

void Controller_multi_planner:: unpause(){
    _paused = false;
}

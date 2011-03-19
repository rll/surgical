#include "controller_visual_knottie.h"
#include <cstdio>

Controller_visual_knottie:: Controller_visual_knottie(int slave_num_, int statesize, int outputsize, const char* matrixfilepath, const char* targetfilepath, double* kp, double* ki, double* kd)
                    : Controller(slave_num_, statesize, outputsize){

    _status = INIT;
    ki[mBASE_RIGHT] = (kp[mBASE_RIGHT]/500.0);
    ki[mBASE_LEFT] = (kp[mBASE_LEFT]/500.0);
    ki[mBASE_REAR] = (kp[mBASE_REAR] /500.0);

    
    _matrixfilepath = matrixfilepath;
    _targetfilepath = targetfilepath;
    
    _geometry = new Accelera_geometry(slave_num_);
    _logfile_override = false;
    sprintf(_logfile_override_msg, "NOT A VALID MESSAGE\n");
    
    int in_port, out_port; 
    if (slave_num == 1)
    {
        in_port = CONTROLLER_IN_PORT_SLAVE1;
        out_port = VISION_IN_PORT_SLAVE1;
    } else {
        in_port = CONTROLLER_IN_PORT_SLAVE2;
        out_port = VISION_IN_PORT_SLAVE2;
    }
    _visual_feedback = new Visual_feedback(in_port, out_port, VISION_IP);


   /* double kp_strong[outputsize];
    double ki_strong[outputsize];
    for (int i = 0; i < outputsize; i++)
    {
        kp_strong[i] = kp[i]*1.1;
        ki_strong[i] = ki[i]*1.2;
    }
*/
    _generic_pid_controller = new Controller_pid(slave_num_, statesize, outputsize, kp, ki, kd);

    /* Crease seperate planned controllers for each motion planning section */
    for (int i=0; i < NUM_SECTIONS_PLANNED; i++)
    {
        _motion_plan_controllers[i] = new Controller_multi_planner(slave_num_, statesize, outputsize, kp, ki, kd, _visual_feedback);
    }

    /* Create seperate LQR controllers for each learned section */
    char sectionmatrixfile[256];
    char sectiontargetfile[256];
    for(int i = 0; i < NUM_SECTIONS_LEARNED; i++){
        sprintf(sectionmatrixfile,"%s%d",matrixfilepath,i+1);
        sprintf(sectiontargetfile,"%s%d",targetfilepath,i+1);    
        _lqr_controllers[i] = new Controller_lqr_and_pid(slave_num_, statesize, outputsize, sectionmatrixfile, sectiontargetfile, kp, ki, kd);
    }
  
   //really should read files to check for this...
   if(slave_num_ == 1){
        _flags[0] = VISION_FLAG;
        _flags[1] = WAIT_FLAG;
    }
    else{
        _flags[0] = WAIT_FLAG;
        _flags[1] = VISION_FLAG;
    }

    _current_lqr_section = -1;
    _current_motion_plan_section = -1;
    _current_flag = -1;
    /* End of lqr reads */   
    

    /* setup the motion planners */
    if (slave_num_ == 1)
    {

        /* begin first segment - pickup */
        double start_state_1[state_size];
        double start_pos_1[num_dof];
        get_last_state_lqr_file(1, start_state_1);
        _geometry->motors_to_point(start_state_1, start_pos_1);
        _motion_plan_controllers[0]->set_start_position(start_pos_1);
        
        add_pickup(_motion_plan_controllers[0]);

        double end_state_1[state_size];
        double end_pos_1[num_dof];
        get_first_state_lqr_file(2, end_state_1);
        _geometry->motors_to_point(end_state_1, end_pos_1);
        _motion_plan_controllers[0]->add_non_visual_segment(3000000, end_pos_1, ABSOLUTE);

        /* end first segment */

        /* begin second segment */

        double start_state_2[state_size];
        double start_pos_2[num_dof];
        get_last_state_lqr_file(2, start_state_2);
        _geometry->motors_to_point(start_state_2, start_pos_2);
        _motion_plan_controllers[1]->set_start_position(start_pos_2);

        double end_state_2[state_size];
        double end_pos_2[num_dof];
        get_first_state_lqr_file(3, end_state_2);
        _geometry->motors_to_point(end_state_2, end_pos_2);
        _motion_plan_controllers[1]->add_non_visual_segment(5000000, end_pos_2, ABSOLUTE);

        /* end second segment */

    } else {
        /* begin first segment - pickup */
        double start_state_1[state_size];
        double start_pos_1[num_dof];
        get_last_state_lqr_file(1, start_state_1);
        _geometry->motors_to_point(start_state_1, start_pos_1);
        _motion_plan_controllers[0]->set_start_position(start_pos_1);
        

        double end_state_1[state_size];
        double end_pos_1[num_dof];
        get_first_state_lqr_file(2, end_state_1);
        _geometry->motors_to_point(end_state_1, end_pos_1);
        _motion_plan_controllers[0]->add_non_visual_segment(5000000, end_pos_1, ABSOLUTE);

        /* end first segment */

        /* begin second segment */

        double start_state_2[state_size];
        double start_pos_2[num_dof];
        get_last_state_lqr_file(2, start_state_2);
        _geometry->motors_to_point(start_state_2, start_pos_2);
        _motion_plan_controllers[1]->set_start_position(start_pos_2);

        add_pickup(_motion_plan_controllers[1]);

        double end_state_2[state_size];
        double end_pos_2[num_dof];
        get_first_state_lqr_file(3, end_state_2);
        _geometry->motors_to_point(end_state_2, end_pos_2);
        _motion_plan_controllers[1]->add_non_visual_segment(3000000, end_pos_2, ABSOLUTE);

        /* end second segment */


    }

    printf("Finished initialization of motion planners for %d", slave_num_);



}

    
Controller_visual_knottie:: ~Controller_visual_knottie(){
    delete _generic_pid_controller;
    delete _geometry;
    delete[] _motion_plan_controllers;
    delete[] _lqr_controllers;
}

int Controller_visual_knottie:: control(const double* state, double* goal, double* out){
    //printf("Currently at state: %f, %f, %f\n",state[mBASE_LEFT],state[mBASE_REAR],state[mBASE_RIGHT]);
    //switch(_status){
    //    case INIT:
    
    int to_return;
    double state_to_hold[state_size];

    switch(_status)
    {
        case INIT:
            printf("Slave %d  is INIT \n", slave_num); 
            get_first_state_lqr_file(1, state_to_hold);
            switch_status( LQR_CONTROL, state, out);
            return hold_position(state, state_to_hold, out);
        break;


        case LQR_CONTROL:
            //printf("Slave %d  is LQR \n", slave_num); 
            if(vision_flag()){
                _logfile_override = true;
                sprintf(_logfile_override_msg, "%%V%%\n");
                _current_flag++;
                printf("Leaving LQR at state: %f, %f, %f\n",state[mBASE_LEFT],state[mBASE_REAR],state[mBASE_RIGHT]);
                return switch_status(MOTION_PLAN, state, out);
            }
            else if(wait_flag()){
               _logfile_override = true;
               sprintf(_logfile_override_msg, "%%W%%\n");
               _current_flag++;
               return switch_status(MOTION_PLAN, state, out);
            }
            else if(is_complete()){
                return switch_status(COMPLETED, state, out);
            }

            _num_iters_since_lqr_start ++;
            to_return = current_lqr_controller()->control(state, goal, out);
            if (_num_iters_since_lqr_start < NUM_ITERS_LQR_TO_AVG) {
                for (int i=0; i < output_size; i++)
                {
                    //double frac = _num_iters_since_lqr_start/NUM_ITERS_LQR_TO_AVG;
                    //out[i] = frac* out[i] + (1-frac)* _last_out[i];                    
                    current_lqr_controller()->reset_time();
                }
            }

            copy_output(out, _last_out);
            return to_return;

        break;
    
        case MOTION_PLAN:
            //printf("Slave %d  is MOTION PLANNING \n", slave_num); 

            double start_state_1[state_size];
            double start_pos_1[num_dof];
            get_last_state_lqr_file(1, start_state_1);
            _geometry->motors_to_point(start_state_1, start_pos_1);

            double curr_pos[num_dof];
            _geometry->motors_to_point(state, curr_pos);
    

            //printf("Last State File: %f, %f, %f, %f, %f, %f, %f\n", start_pos_1[iX], start_pos_1[iY], start_pos_1[iZ], start_pos_1[iPITCH], start_pos_1[iTILT], start_pos_1[iGROSS], start_pos_1[iGRIP]);
            //printf("Current State: %f, %f, %f, %f, %f, %f, %f\n", curr_pos[iX], curr_pos[iY], curr_pos[iZ], curr_pos[iPITCH], curr_pos[iTILT], curr_pos[iGROSS], curr_pos[iGRIP]);


            if (!current_motion_plan_controller()->is_complete())
            {
                int toReturn = current_motion_plan_controller()->control(state, goal, out);
                current_motion_plan_controller()->get_last_target(_last_target);
                copy_output(out, _last_out);
                
                //printf("Motion Planning To: %f, %f, %f, %f, %f, %f, %f\n", _last_target[iX], _last_target[iY], _last_target[iZ], _last_target[iPITCH], _last_target[iTILT], _last_target[iGROSS], _last_target[iGRIP]);
        
                //printf("With Final Goal Position: %f, %f, %f\n",_goal_pos[iX],_goal_pos[iY],_goal_pos[iZ]);
                //printf("With Offset: %f, %f, %f\n",_traj_pos_offsets[iX],_traj_pos_offsets[iY],_traj_pos_offsets[iZ]);
                //_status = MOTION_PLAN;   //don't know why this has to be here...
                return toReturn;
            }
            else
            {
                printf("Slave %d reported that it finished \n", slave_num);
                current_motion_plan_controller()->get_last_target(_last_target);
                state_to_hold [state_size];
                _geometry->point_to_motors(_last_target, state_to_hold);
                if (_flags[_current_flag] == VISION_FLAG)
                {
                    if(_other_slave){
                        printf(" Slave %d   is resuming the other slave \n", slave_num);
                        _other_slave->resume();
                    }

                    switch_status(LQR_CONTROL, state_to_hold, out);
                 } else {
                    switch_status(WAITING, state_to_hold, out);   
                 }
                 return current_motion_plan_controller()->hold_position(state, state_to_hold, out);
            }
        break;

        case WAITING:
             //printf("Slave %d  is WAITING \n", slave_num); 
             state_to_hold [state_size];
             _geometry->point_to_motors(_last_target, state_to_hold);
             //return hold_position(state, state_to_hold, out);        
             to_return = current_motion_plan_controller()->hold_position(state, state_to_hold, out);
             copy_output(out, _last_out);
             return to_return;
        break;

        case COMPLETED:
             //printf("Slave %d  is COMPLETED \n", slave_num); 
             state_to_hold [state_size];
             _geometry->point_to_motors(_last_target, state_to_hold);
             return hold_position(state, state_to_hold, out);        
        break;
    }
                   
    
}




int Controller_visual_knottie:: switch_status(Status status, const double* state, double* out){
    _status = status;

     if(status == LQR_CONTROL){
        _current_lqr_section++;
        current_lqr_controller()->reset_time();
        _num_iters_since_lqr_start = 0;
     }
     if (status == MOTION_PLAN){
        _current_motion_plan_section++;
        current_motion_plan_controller()->init_voltage(_last_out);
     }

     switch(status){
        case LQR_CONTROL:
            printf("Slave %d Beginning LQR Control\n", slave_num);
        break;
        
        case MOTION_PLAN:
            printf("Slave %d Beginning Motion Planning \n", slave_num);
        break;
        
        case COMPLETED:
            printf("Slave %d Completed!\n", slave_num);
        break;
        
        case WAITING:
            printf("Slave %d initiating Wait Period\n", slave_num);
        break;
     
     }
    
     //for smoothness, just output whatever we previously output
     //copy_output(_last_out, out);
     //return output_size;
     
     double state_to_hold [state_size];
     _geometry->point_to_motors(_last_target, state_to_hold);

    //return hold_position(state, out);
}


void Controller_visual_knottie::add_pickup(Controller_multi_planner* motion_plan_controller)
{
    double vis_offset_1[num_dof];
    zero_state(vis_offset_1);
    vis_offset_1[iZ] = 7;

    std::vector<double> servo_times_1 = std::vector<double>();
    servo_times_1.push_back(200000.0);
    servo_times_1.push_back(2800000.0);
    motion_plan_controller->add_visual_segment(3000000.0, servo_times_1, vis_offset_1);
    motion_plan_controller->open_grip();

    double rel_pos_1[num_dof]; zero_state(rel_pos_1);
    rel_pos_1[iZ] = -17.0; 
    motion_plan_controller->add_non_visual_segment(1000000, rel_pos_1);
    
    motion_plan_controller->close_grip();
    
    double rel_pos_2[num_dof]; zero_state(rel_pos_2);
    rel_pos_2[iZ] = 12.0; 
    motion_plan_controller->add_non_visual_segment(1500000, rel_pos_2);

}






void Controller_visual_knottie:: copy_state(const double* original, double* copy){
    for(int i = 0; i < state_size; i++){
        copy[i] = original[i];   
    }
}

void Controller_visual_knottie:: copy_output(const double* original, double* copy){
    for(int i = 0; i < output_size; i++){
        copy[i] = original[i];   
    }
}

void Controller_visual_knottie:: zero_state(double* state)
{
    for(int i = 0; i < state_size; i++){
        state[i] = 0.0;   
    }
}

int  Controller_visual_knottie:: hold_position(const double* state, const double* state_to_hold, double* out){
    double goal[state_size];
    copy_state(state_to_hold,goal);
    return _generic_pid_controller->control(state, goal, out);   
}


Controller_lqr_and_pid* Controller_visual_knottie:: current_lqr_controller()
{
    return _lqr_controllers[_current_lqr_section];
}

Controller_multi_planner* Controller_visual_knottie:: current_motion_plan_controller()
{
    return _motion_plan_controllers[_current_motion_plan_section];
}


Controller_visual_knottie::Flags Controller_visual_knottie:: next_flag(){
    if (_current_flag >= NUM_SECTIONS_LEARNED - 2)
        return NO_FLAG;
    return _flags[_current_flag+1];
}


bool Controller_visual_knottie:: vision_flag()
{
    return current_lqr_controller()->is_complete() && next_flag() == VISION_FLAG;
}

bool Controller_visual_knottie:: wait_flag()
{
    return current_lqr_controller()->is_complete() && next_flag() == WAIT_FLAG;
}



/** Gives me a pointer to another slave controller, which I am responsible for making wait and resume the moment I see a flag */
void Controller_visual_knottie:: set_other_controller(boost::shared_ptr<Controller> controller){
    _other_slave = boost::shared_dynamic_cast<Controller_visual_knottie>(controller);
}

/** Called by another slave controller, to tell me to stop waiting once it has done its job*/
void Controller_visual_knottie:: resume(){
    double state[state_size]; //Won't actually be used for anything, just to appease the switch_status gods.
    switch_status(LQR_CONTROL, state, _last_out);
}

/* If we are running anything but LQR control, don't log! */
bool Controller_visual_knottie:: ignore_log(){
    return _status != LQR_CONTROL;
}

bool Controller_visual_knottie:: log_message(char* msg){
    if(_logfile_override){
        strcpy(msg,_logfile_override_msg);
        _logfile_override = false;
        return true;
    }
    return false;
}

void Controller_visual_knottie:: get_first_state_lqr_file(int lqr_file_num, double* data){
    char sectiontargetfile[256];
    sprintf(sectiontargetfile,"%s%d",_targetfilepath, lqr_file_num);
    std::string s;
    std::ifstream in;
    in.open(sectiontargetfile);
    double trash;
    in >> trash; //toss out timestamp
    for(int i=0;i<num_actuators;i++) {
        in >> data[i];
    }
}


void Controller_visual_knottie:: get_last_state_lqr_file(int lqr_file_num, double* data){
    char sectiontargetfile[256];
    sprintf(sectiontargetfile,"%s%d", _targetfilepath, lqr_file_num);

    int lineSize = 10000;
    char buf[lineSize+1];
    char buf2[lineSize+1];
    FILE *fp;
    fp = fopen(sectiontargetfile, "r");
    fseek(fp, -lineSize, SEEK_END);
    fread(buf, sizeof(char), lineSize, fp);
    buf[lineSize] = '\0';
    buf[lineSize-1] = '\0';
    fclose(fp);
    char* ptr = strrchr(buf, '\n');

    strcpy(buf2, ptr);

    char* tokens = strtok(buf2, " \n\t_,");
    
    //data[0] = atof(tokens);
    tokens = strtok(NULL, " ");
    for (int i=0; i < num_actuators; i++)
    {
        data[i] = atof(tokens);
        tokens = strtok(NULL, " ");
    }


}


bool Controller_visual_knottie:: is_complete(){
     return current_lqr_controller()->is_complete() && next_flag() == NO_FLAG;
}






/** Reads the current target file and notifies in the event of a vision flag */
/*bool Controller_visual:: vision_flag(){
    return current_lqr_controller()->is_complete() && next_flag() == VISION_FLAG;
    
    
    //_target_data->seekg(_standard_controller->base_controller->target_data->tellg());
    //std::cout << _standard_controller->base_controller->target_data->tellg();
    //std::cout << _standard_controller->base_controller->target_data->tellg() << std::endl;
    //char str[20];
    //_target_data->getline(str, 20);
    
    /*
    timeval now;
    gettimeofday(&now, NULL);
    if(!_flag_times.empty() && _flag_times.front().flag == VISION_FLAG && timediff(now, _lqr_start_time) >= _flag_times.front().time){
        _flag_times.pop_front();
       
        
        _target_data->reset_time();
        std::vector<double> data = _standard_controller->base_controller->target_data->data();
        for(int i = 0; i < num_actuators; i++){
            _return_state[i] = data[i];
        }
        _standard_controller->base_controller->matrix_data->data(); //Just to empty and make sure we're at 0
        return true;
    }
    return false;
    */
//}

/** Reads the current target file and notifies in the event of a wait flag */
/*bool Controller_visual:: wait_flag(){
    return current_lqr_controller()->is_complete() && next_flag() == WAIT_FLAG;
    
    /*
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
    */
//}



/** Checks if given string is a vision flag */
bool Controller_visual_knottie:: isVisionFlag(const char* str){
    if(str[0] == '%' && str[1] == 'V'){
        return true;
    }
    else
        return false;
}

/** Checks if given string is a vision flag */
bool Controller_visual_knottie:: isWaitFlag(const char* str){
    if(str[0] == '%' && str[1] == 'W')
        return true;
    else
        return false;
}




/*void    Controller_visual:: pause_here(const double* state){
    copy_state(state, _hold_state);
    //printf("Pausing at: %f, %f, %f\n",_hold_state[mBASE_LEFT],_hold_state[mBASE_REAR],_hold_state[mBASE_RIGHT]);
}*/

/** Gives me a pointer to another slave controller, which I am responsible for making wait and resume the moment I see a flag */
/*void Controller_visual:: set_other_controller(boost::shared_ptr<Controller> controller){
    _other_slave = boost::shared_dynamic_cast<Controller_visual>(controller);
}*/

/** Called by another slave controller, to tell me to stop waiting once it has done its job*/
/*void Controller_visual:: resume(){
    double out[num_actuators]; //Won't actually be used for anything, just to appease the switch_status gods.
    switch_status(LQR_CONTROL,_hold_state,out);
}*/

/* If we are running anything but LQR control, don't log! */
/*bool Controller_visual:: ignore_log(){
    return _status != LQR_CONTROL;
}*/

/*bool Controller_visual:: log_message(char* msg){
    if(_logfile_override){
        strcpy(msg,_logfile_override_msg);
        _logfile_override = false;
    }
}*/

/* Sets the "grip" to be at the current state, minus the offset, plus grip */
/*void Controller_visual:: create_grip_state(const double* state, double* grip_state){
    double position[num_dof];
    double grip_position[num_dof];
    _geometry->motors_to_point(state, position);
    for(int i = 0; i < num_dof; i++){
        grip_position[i] = position[i] - _goal_pos_offset[i];
    }
    grip_position[iGRIP] = GRIP_ON;
    _geometry->point_to_motors(grip_position, grip_state);
}
*/
/* Sets an ungrip state at the current state */
/*void Controller_visual:: create_ungrip_state(const double* state, double* ungrip_state){
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

Controller_lqr_and_pid* Controller_visual:: current_lqr_controller(){
    return _lqr_controllers[_current_section];
}

Controller_visual::Flags Controller_visual:: next_flag(){
    if (_current_flag >= NUM_SECTIONS - 2)
        return NO_FLAG;
    return _flags[_current_flag+1];
}

void Controller_visual:: get_return_state(){
    char sectiontargetfile[256];
    /* NOTE: Call AFTER you have incremented current_section */
/*    sprintf(sectiontargetfile,"%s%d",_targetfilepath,_current_section+1);
    Time_file* target = new Time_file(sectiontargetfile,num_actuators);
    std::vector<double> data = target->data();
    for(int i = 0; i < num_actuators; i++){
        _return_state[i] = data[i];
    }   
}

bool Controller_visual:: is_complete(){
     return current_lqr_controller()->is_complete() && next_flag() == NO_FLAG;
}*/

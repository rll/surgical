#include "slave.h"

using std::cout;
using std::endl;

Slave::Slave(int id, boost::shared_ptr<Controller> controller, boost::shared_ptr<System> s) {
    int i;
    _id = id;
    _contr              = controller;
    _sys                = s;
    _interval           = 1000;
    reset_timer();

}

Slave::~Slave() {
    int i;

    if (_logfile.is_open())
        _logfile.close();

    _sys->reset_all();
}

void Slave::init_setpoint(int index, double value){
    _sys->set_position(index, value);
    _sys->queue_voltage(index, 0.0);
    _sys->apply_voltages();
}

void Slave::move_to(double pos[num_dof], bool use_kdl, bool block){
    int i;
    if( update_now() ) {
        // Obtain nearest legal point
        double safe[num_dof];
        double pose[num_dof];
        for (int i = 0; i < num_dof; i++) {
            pose[i] = pos[i];
        }
        if(use_kdl) { 
            double motor_pos[num_actuators];
           
            if(point_to_motors(pose, motor_pos, false, true) == NULL) { 
                cout << "Error: KDL Did not Converge. Will not move\n";
                return; 
            }
            motors_to_point(motor_pos, pose);
        }
        nearest_legal_point(pose, safe);
        if(_id == 1 && safe[iGRIP] > 0){
            safe[iGRIP] *= SLAVE_1_GRIP_FACTOR;
            safe[iGRIP] += fabs(safe[iTILT] - M_PI/2)*SLAVE_1_TILT_COMPENSATION;
        }
        else{
            safe[iGRIP] *= SLAVE_2_GRIP_FACTOR;
            safe[iGRIP] += fabs(safe[iTILT] - M_PI/2)*SLAVE_2_TILT_COMPENSATION;
        }
        double motor_positions[num_actuators];
        point_to_motors(safe, motor_positions, false, false);
        reset_timer();
        
        if(PRINT_GRIP_POS_VS_GOAL){
            double result[num_dof];
            current_pose(result);
            debugprintf("Current grip position is: %f\n", result[6]);
            debugprintf("Current grip goal is: %f\n", pos[6]);
            debugprintf("Current safe goal is: %f\n", safe[6]);
        }
        
        // Voltage out
        motors_to(motor_positions, block);
    }
    
    
}

void Slave::motors_to(double motor_positions[num_actuators], bool block){
    double state[num_actuators];
    double voltages[num_actuators];
    double motor_pos[num_actuators];
    for (int channel = 0; channel < num_actuators; channel++) { 
        motor_pos[channel] = motor_positions[channel]; 
    }   
    int i;
    _sys->read_encoders(state);
    _contr->control(state, motor_positions, voltages);

    // logging
    if(_logfile.is_open() && !_contr->ignore_log()) {
        timeval t;
        gettimeofday(&t, NULL);

        // <slave_num> <time>:
        double actual_time = (double)t.tv_sec;
        actual_time *= 1000000.0;
        actual_time += (double)t.tv_usec;
        _logfile << _id << " " << actual_time << " ";

        // encoder values
        double encoders[num_actuators];
        current_position(encoders);
        for(i=0;i<num_actuators;i++) {
            _logfile << encoders[i] << " ";
        }

        // voltages
        for(i=0;i<num_actuators;i++) {
            _logfile << voltages[i] << " ";
        }
        _logfile << std::endl;
    }
    char buf[256];
    if(_logfile.is_open() && _contr->log_message(buf)){
        printf("should be logging message %s \n", buf);
        _logfile << buf;
    }

    _sys->queue_voltages(voltages);
    _sys->apply_voltages();
    _sys->set_reference_position(motor_pos);
    
    if(block) {
        block_while_moving();
    }
}

double* Slave::current_pose(double result[num_dof], bool use_elbow_coor, bool use_kdl, bool no_cache) {
    double args[num_actuators];
    _sys->read_encoders(args, no_cache);
    motors_to_point(args, result, use_elbow_coor, use_kdl);
    return result;
}

double* Slave::current_position(double result[num_actuators], bool no_cache){
    _sys->read_encoders(result, no_cache);
    return result;
}

double* Slave::current_velocity(double result[num_actuators], bool no_cache) {
    _sys->read_velocities(result, no_cache);
    return result;
}

void Slave::block_while_moving() { 
    bool moving = true;
    double motor_vel_read[num_actuators];
    int v_0_count = 0;
    while (moving || v_0_count != 1000) { 
        sleep(0.01);
        current_velocity(motor_vel_read, true);
        moving = false; 
        for(int i = 0; i < num_actuators; i++) {
            if(motor_vel_read[i] != 0) { 
                moving = true;
                v_0_count = 990;
            }
        }
        if (!moving) {
            v_0_count += 1;
        }
    }
}


bool Slave::update_now() {
    timeval now;
    gettimeofday(&now, NULL);
    
    time_t diff = (now.tv_sec * 1000000 + now.tv_usec) - (_previous_time.tv_sec * 1000000 + _previous_time.tv_usec);
    return diff > _interval;
}

void Slave::reset_timer() {
    gettimeofday(&_previous_time, NULL);
}

void Slave::start_logging() {
    char buf[30];
    std::string ofile_string = std::string(LOGGING_PREFIX);
    sprintf(buf, "slave_%d__", _id);
    ofile_string += std::string(buf);

    std::cout << ofile_string << std::endl;
    char* ofile_name = time_stamped_string(ofile_string.c_str());
    std::cout << ofile_name << std::endl;
    _logfile.open(ofile_name);
    free(ofile_name);
    _logfile.precision(25);
}

void Slave::stop_logging() {
    if (_logfile.is_open()) {
        _logfile.close();
        std::cout << "logging stopped" << std::endl;
    }
}

void Slave::log(const char* msg){
    _logfile << msg;
}

void Slave::print() {
    double pos[num_dof];
    double motor_pos[num_actuators];
    current_position(motor_pos);
    motors_to_point(motor_pos, pos, false, true);
    //current_pose(pos);
    printf("\nslave %d: ",_id);
    for(int i=0;i<num_dof;i++) {
        printf("[%d] %f, ", i, pos[i]);
    }

    printf("\nmotor %d: ", _id);
    double position[num_actuators];
    current_position(position);
    for(int i=0;i<num_actuators;i++) {
        printf("[%d] %f, ", i, position[i]);
    }
    printf("\n");
}

#include "controller_lqr_and_pid.h"

Controller_lqr_and_pid::Controller_lqr_and_pid(int slave, int statesize, int outputsize, const char* matrixfilepath, const char* targetfilepath, double* kp, double* ki, double* kd) : Controller(slave, statesize, outputsize) {
    base_controller = new Controller_lqr(slave, statesize, outputsize, matrixfilepath, targetfilepath);
    wrist_controller = new Controller_pid(slave, statesize, outputsize, kp, ki, kd);
    _geometry = new Accelera_geometry(slave);
}

Controller_lqr_and_pid::~Controller_lqr_and_pid() {
    delete base_controller;
    delete wrist_controller;
}

// state = [position; velocity; total volt]
int Controller_lqr_and_pid::control(const double* state, double* goal, double* out) {
    double wristout[8];
    double baseout[8];
    int outputsize;
    //double gripValue = goal[0];
    //double pitchValue = goal[1];
    //double grossValue = goal[2];
    double gripValue = 0;
    double pitchValue = 0;
    double grossValue = 0;
    
    outputsize = base_controller->control(state, goal, baseout); // Goal overwritten, populated by LQR target
    for(int i = 0; i < num_actuators; i++){
        out[i] = 0.0;
    }
    goal[0] += 0.35 * gripValue;
    goal[2] -= 0.35 * gripValue;
    
    goal[0] += 1.005 * pitchValue/4;
    goal[1] += 0.680 * pitchValue/4;
    goal[2] += 0.680 * pitchValue/4;
    
    goal[3] += 0.673 * grossValue/2;
    double state_cp[num_actuators];
    for(int i = 0; i < num_actuators; i++){
        state_cp[i] = state[i];
    }
    double position[num_dof];
    _geometry->motors_to_point(state, position);
    position[iGRIP] *= 1.15;
    double wrist_state[num_actuators];
    _geometry->point_to_motors(position, wrist_state);
    state_cp[mYAW1] = wrist_state[mYAW1];
    state_cp[mYAW2] = wrist_state[mYAW2];
    wrist_controller->control(wrist_state, goal, wristout);
    
    
    out[mPITCH] = wristout[mPITCH];
    out[mGROSS] = wristout[mGROSS];
    out[mYAW1] = wristout[mYAW1];
    out[mYAW2] = wristout[mYAW2];
    out[mBASE_REAR] = baseout[mBASE_REAR];
    out[mBASE_LEFT] = baseout[mBASE_LEFT];
    out[mBASE_RIGHT] = baseout[mBASE_RIGHT];
    return outputsize;
}

void Controller_lqr_and_pid:: reset_time(){
    base_controller->reset_time();
}

bool Controller_lqr_and_pid:: is_complete(){
    return base_controller->is_complete();
}


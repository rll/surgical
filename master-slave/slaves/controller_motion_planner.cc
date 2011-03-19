#include "controller_motion_planner.h"

/* Takes STATES, not Positions */
Controller_motion_planner::Controller_motion_planner(int slave_num, int statesize, int outputsize, double* kp, double* ki, double* kd, const double* start_state, const double* goal_state, double duration) : Controller(slave_num, statesize, outputsize) {
    _geometry = new Accelera_geometry(slave_num);
    double start_pos[num_dof];
    _geometry->motors_to_point(start_state, start_pos);
    double goal_pos[num_dof];
    _geometry->motors_to_point(goal_state, goal_pos);
    _pid_controller = new Controller_pid(slave_num, statesize, outputsize, kp, ki, kd);
    _motion_planner = new Interpolate_planner(slave_num, num_dof, start_pos, goal_pos, duration);
    _motion_planner->begin();
}
    
Controller_motion_planner::~Controller_motion_planner() {
    delete _geometry;
    delete _motion_planner;
    delete _pid_controller;
}

int Controller_motion_planner::control(const double* state, double* goal, double* out) {
    double next_pos[num_dof];
    double cur_pos[num_dof];
    _geometry->motors_to_point(state, cur_pos);
    
    _motion_planner->get_next_state(cur_pos, next_pos);
    double next_state[state_size];
    _geometry->point_to_motors(next_pos, next_state);
    return _pid_controller->control(state, next_state, out);
}

bool Controller_motion_planner::is_complete(const double* state){
    double position[num_dof];
    _geometry->motors_to_point(state, position);
    return _motion_planner->is_complete(position);
}



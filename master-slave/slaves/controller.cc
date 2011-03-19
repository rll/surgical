#include "controller.h"

Controller::Controller(int slave, int statesize, int outputsize){
    slave_num   = slave;
    state_size  = statesize;
    output_size = outputsize;
}

Controller::~Controller() { }

bool Controller::ignore_log(){
    return false; //By default, we have no reason to ever not log our motion
}

/* If there is an override to add to the log, return "true" and populate msg accordingly */
bool Controller::log_message(char* msg){
    return false;
}

void Controller::set_other_controller(boost::shared_ptr<Controller> controller){
    return;
}

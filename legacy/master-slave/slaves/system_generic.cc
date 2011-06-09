#include "system_generic.h"

System_generic::System_generic(int slave, Actuator** actuators, Sensor** sensors) : System(slave) {
    _actuators  = actuators;
    _sensors    = sensors;
    
    reset_all();
}

System_generic::~System_generic() {
    for(int i=0;i<num_actuators;i++) {
        delete _actuators[i];
        delete _sensors[i];
    }
    delete[] _actuators;
    delete[] _sensors;
}

void System_generic::init_encoder(int channel, int sign, double ratio, double offset) {
    _sensors[channel]->channel    = channel;
    _sensors[channel]->sign       = (float) sign;
    _sensors[channel]->ratio      = ratio;
    _sensors[channel]->offset     = offset;

    _sensors[channel]->Init(0);
}

void System_generic::set_position(int channel, double value) {
    _sensors[channel]->Init(value);
}

double System_generic::read_encoder(int channel, bool no_cache) {
    return _sensors[channel]->Input();
}

double System_generic::read_velocity(int channel, bool no_cache) {
    return 0;
}

void System_generic::init_pid(int channel) { 
}

void System_generic::init_reference_position() {
}

void System_generic::set_reference_position(double motor_pos[num_actuators]) {
}

void System_generic::init_pid_control() {
}

void System_generic::init_actuator(int channel, int sign, double offset) {
    _actuators[channel]->m_channel    = channel;
    _actuators[channel]->m_sign       = sign;
    _actuators[channel]->m_offset     = offset;
    _actuators[channel]->Output(0.0);
}

void System_generic::queue_voltage(int channel, double volt) {
    queued_voltages[channel] = volt;
}

void System_generic::apply_voltages() {
    for(int i=0;i<num_actuators;i++) {
        _actuators[i]->Output(queued_voltages[i]);
    }
}

void System_generic::set_limit(int channel, double min, double max) {
    _actuators[channel]->setLimit(min, max);
}

void System_generic::set_torque_limit(int channel, double limit, double peak){
    _actuators[channel]->setLimit(-1*limit,limit);
}

void System_generic::reset_motors(){
    reset_all();
}


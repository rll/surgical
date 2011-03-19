#include "system_mark1.h"

using std::cout;
using std::endl;

System_mark1::System_mark1(int slave, std::map<int,int> encoder_actuator) : System(slave) { 
    motor_channel = encoder_actuator;    
}

System_mark1::~System_mark1() { }

void System_mark1::init_encoder(int channel, int sign, double ratio, double offset) {
    assert( channel < num_actuators && channel >= 0);

    encoder_signs[channel] = sign;
    encoder_ratios[channel] = ratio;
    encoder_offsets[channel] = offset;

	miranova_init_channel(channel + (slave_num-1)*num_actuators, 0x7FFFFFL, 4);
}

double System_mark1::read_encoder(int channel) {
    assert( channel < num_actuators && channel >= 0);
    int status;
	long val = miranova_read(channel + (slave_num-1)*num_sensors, status) - 0x7FFFFFL;

    // Convert to our units
	double currentInput =  val / encoder_ratios[channel];
	currentInput *= encoder_signs[channel];
	currentInput += encoder_offsets[channel];
    return currentInput;
}

void System_mark1::set_position(int channel, double value) {
    init_encoder(channel, encoder_signs[channel], encoder_ratios[channel], value);
}

void System_mark1::init_actuator(int channel, int sign, double offset) {
    channel = motor_channel[channel];
    assert( channel < num_actuators && channel >= 0);
    actuator_signs[channel] = sign;
    actuator_offsets[channel] = offset;

    getperm(QUATECH_DABASE1,4);
    getperm(QUATECH_DABASE2,4);
    outb(0x80, QUATECH_DABASE1 + 3);              // setup the address
    outb(0x80, QUATECH_DABASE2 + 3);              // setup the address
}

void System_mark1::queue_voltage(int channel, double volt) {

    channel = motor_channel[channel];
    assert( channel < num_actuators && channel >= 0);
    if(volt > actuator_maxs[channel]) 
        volt = actuator_maxs[channel];
    else if(volt < actuator_mins[channel]) 
        volt = actuator_mins[channel];

    // Sign + offset
    volt *= actuator_signs[channel];
    volt += actuator_offsets[channel];

    queued_voltages[channel] = volt;
}

void System_mark1::apply_voltages() {
    for(int i=0;i<num_actuators;i++) {
	    quatech_vout(i, actuator_offsets[i]+queued_voltages[i], slave_num);
    }
}

void System_mark1::set_limit(int channel, double min, double max) {
    channel = motor_channel[channel];
    assert( channel < num_actuators && channel >= 0);
    actuator_mins[channel] = min;
    actuator_maxs[channel] = max;
}

void System_mark1::reset_motors(){
    reset_all();
}

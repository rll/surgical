#include "system.h"
//[b,a] = cheby2(5,20,.2)
double b[6] = { 0.084226357716145,  -0.138612396592324,   0.087352881630195,   0.087352881630195,  -0.138612396592324,   0.084226357716146 };
double a[6] = { 1.000000000000000,  -2.966123533950475,   3.815765378671260,  -2.543840645716643,   0.878222628350575,  -0.118090141846685 };

System::System(int slave) {
    slave_num = slave;
    for(int i=0;i<num_actuators;i++) {
        filters[i]          = new LowPassFilter(a, b, 6, 1);
        last_encoders[i]    = 0.0;
        timers[i]           = new Timer();
    }
}

System::~System() { 
    reset_all();
}

void System::init_encoders(double info[num_actuators][3]) {
    for(int i=0;i<num_actuators;i++) {
        init_encoder(i, (int) info[i][0], info[i][1], info[i][2]);
        last_encoders[i] = 0.0;
    }
}

double* System::read_encoders( double out[num_actuators], bool no_cache) {
    for(int i=0;i<num_actuators;i++) {
        out[i] = read_encoder(i, no_cache);
    }
    return out;
}

/*double System::read_velocity(int channel) {
    double current_position = read_encoder(channel);
    double velocity = (current_position - last_encoders[channel]) / (timers[channel]->dt());
    velocity *= 1000000;    // Put into mm/sec or rad/sec
    double out;
    filters[channel]->Update(&velocity, &out);
    last_encoders[channel] = current_position;
    return out;
}*/

double* System::read_velocities(double out[num_actuators], bool no_cache) {
    for(int i=0;i<num_actuators;i++) {
        out[i] = read_velocity(i, no_cache);
    }
    return out;
}

void System::init_actuators(double info[num_actuators][2]) {
    for(int i=0;i<num_actuators;i++) {
        init_actuator(i, (int) info[i][0], info[i][1]);
    }
}

void System::queue_voltages(double volts[num_actuators]) {
    for(int i=0;i<num_actuators;i++) {
        queue_voltage(i, volts[i]);
    }
}

void System::set_limits(double info[num_actuators][2]) {
    for(int i=0;i<num_actuators;i++) {
        set_limit(i, info[i][0], info[i][1]);
    }
}

void System::reset_all() {
    for(int i=0;i<num_actuators;i++) {
        queue_voltage(i, 0.0);
    }
    apply_voltages();
}

void System::reset_motors() {
    printf("CALLED RESET_MOTORS ON SYSTEM CLASS\n");
    reset_all();
}

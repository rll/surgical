#ifndef _SYSTEM_GENERIC_H_
#define _SYSTEM_GENERIC_H_
#include "shared.h"
#include "actuator.h"
#include "sensor.h"
#include "system.h"

class System_generic : public System {
    public:

    Actuator**          _actuators;
    Sensor**            _sensors;

    System_generic(int slave, Actuator** actuators, Sensor** sensors);
    ~System_generic();

    ////////////////////////////////////////
    ////         Encoder Related        ////
    ////////////////////////////////////////

    void        init_encoder(int channel, int sign, double ratio, double offset);
    void        set_position(int channel, double value);
    double      read_encoder(int channel, bool no_cache=false);
    double      read_velocity(int channel, bool no_cache=false);
    
    void        init_pid_control();
    void        init_pid(int channel);
    void        init_reference_position();
    void        set_reference_position(double motor_pos[num_actuators]);
    ////////////////////////////////////////
    ////        Actuator Related        ////
    ////////////////////////////////////////

    void        init_actuator(int channel, int sign, double offset);
    void        queue_voltage(int channel, double volt);
    void        apply_voltages();
    void        set_limit(int channel, double min, double max);
    void        set_torque_limit(int channel, double limit, double peak);
    
    void        reset_motors();
};
#endif

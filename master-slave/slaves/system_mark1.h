#ifndef     _SYSTEM_MARK1_H_
#define     _SYSTEM_MARK1_H_

#include "shared.h"
#include "system.h"
#include "util.h"
#include "LowPassFilter.h"

class System_mark1 : public System {
    public:

    std::map<int,int> motor_channel;

    System_mark1(int slave, std::map<int,int> encoder_actuator_map);
    ~System_mark1();

    ////////////////////////////////////////
    ////         Encoder Related        ////
    ////////////////////////////////////////

    int         encoder_signs[num_actuators];
    double      encoder_ratios[num_actuators];
    double      encoder_offsets[num_actuators];

    void        init_encoder(int channel, int sign, double ratio, double offset);
    void        set_position(int channel, double value);
    double      read_encoder(int channel);

    ////////////////////////////////////////
    ////        Actuator Related        ////
    ////////////////////////////////////////

    int         actuator_signs[num_actuators];
    double      actuator_offsets[num_actuators];
    double      actuator_mins[num_actuators];
    double      actuator_maxs[num_actuators];

    void        init_actuator(int channel, int sign, double offset);
    void        queue_voltage(int channel, double volt);
    void        apply_voltages();
    void        set_limit(int channel, double min, double max);
    void        reset_motors();
};

#endif

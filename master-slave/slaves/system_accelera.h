#ifndef _SYSTEM_ACCELERA_H_
#define _SYSTEM_ACCELERA_H_
#include "shared.h"
#include "util.h"
#include "system.h"

class System_accelera : public System{
    public:
    
    boost::shared_ptr<Galil> g;
    double maxVoltage[num_actuators];
    int slave_id;

    System_accelera(int slave, boost::shared_ptr<Galil> galil);
    ~System_accelera();
    

    ////////////////////////////////////////
    ////         Encoder Related        ////
    ////////////////////////////////////////

    int         encoder_signs[num_actuators];
    double      encoder_ratios[num_actuators];
    double      encoder_offsets[num_actuators];
    double      pid_constants[num_actuators][3];

    void        init_encoder(int channel, int sign, double ratio, double offset);
    double      read_encoder(int channel, bool no_cache=false);
    void        set_position(int channel, double value);
    
    double      read_velocity(int channel, bool no_cache=false);
    
    // PID related
    void        init_pid_control();
    void        init_pid(int channel);
    void        init_reference_position();
    void        set_reference_position(double motor_pos[num_actuators]);

    // Galil related
    void        flush_galil_recv_buffer();

    std::vector<char> latest_record;
    void        fetch_record();

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
    void        set_torque_limit(int channel, double limit, double peak);
    
    //Added for testing purposes//
    void        send_command(const char* command);
    void        send_command(std::string& command);
    void        reset_motors();
};
#endif

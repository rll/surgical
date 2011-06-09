#ifndef     _SYSTEM_H_
#define     _SYSTEM_H_
#include "shared.h"
#include "util.h"
#include "LowPassFilter.h"

class System {
    public:

    int             slave_num;
    LowPassFilter*  filters[num_actuators];
    double          last_encoders[num_actuators];
    Timer*          timers[num_actuators];
    double          queued_voltages[num_actuators];

    System(int slave);
    ~System();

    ////////////////////////////////////////
    ////         Encoder Related        ////
    ////////////////////////////////////////

    virtual void        init_encoder(int channel, int sign, double ratio, double offset)=0;
    void                init_encoders(double info[num_actuators][3]);
    
    virtual void        init_pid_control()=0;
    virtual void        init_pid(int channel)=0;
    virtual void        init_reference_position()=0;
    virtual void        set_reference_position(double motor_pos[num_actuators])=0;

    virtual void        set_position(int channel, double value)=0;

    virtual double      read_encoder(int channel, bool no_cache=false)=0;
    double*             read_encoders(double out[num_actuators], bool no_cache=false);

    virtual double      read_velocity(int channel, bool no_cache=false)=0;
    double*             read_velocities(double out[num_actuators], bool no_cache=false);

    ////////////////////////////////////////
    ////        Actuator Related        ////
    ////////////////////////////////////////

    virtual void        init_actuator(int channel, int sign, double offset)=0;
    void                init_actuators(double info[num_actuators][2]);

    virtual void        queue_voltage(int channel, double volt)=0;
    void                queue_voltages(double volts[num_actuators]);

    virtual void        apply_voltages()=0;
    void                reset_all();

    virtual void        set_limit(int channel, double min, double max)=0;
    void                set_limits(double info[num_actuators][2]);
    
    virtual void        set_torque_limit(int channel, double limit, double peak)=0;
    
    virtual void        reset_motors()=0;
};
#endif

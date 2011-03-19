#include "shared.h"
#include "imports.h"
#include "controller_visual.h"
#include "controller_visual_servo.h"
#include "controller_visual_knottie.h"

#define MODE1 PID
#define NUMSERVOS1 1
#define SERVODURATION1 5000000

using namespace std;
    
Slave_mark1* CreateSlave1_accelera()
{
    short i;
    int slave_num = 1;

    boost::shared_ptr<Galil>        g(new Galil(SLAVE1_IP));
    boost::shared_ptr<System_accelera>       sys(new System_accelera(slave_num,g));

    // choose sensors
    std::cout << "initiallizing encoders...";
    sys->init_encoder(0,  1,  R_COUNT_WRIST, 0);
    sys->init_encoder(1,  1,  R_COUNT_WRIST, 0);
    sys->init_encoder(2,  1,  R_COUNT_WRIST, 0);
    sys->init_encoder(3,  1,  R_COUNT_WRIST, 0);
    sys->init_encoder(4, -1,  M_COUNT, 0);
    sys->init_encoder(5, -1,  M_COUNT, 0);
    sys->init_encoder(6, -1,  M_COUNT, 0);
    sys->init_encoder(7, -1,  M_COUNT, 0);
    std::cout << "done!" << std::endl;

    // choose actuators
    std::cout << "initiallizing actuators...";
    double actuator_offsets[num_actuators] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    sys->init_actuator(0, 1, actuator_offsets[0]);
    sys->init_actuator(1, 1, actuator_offsets[1]);
    sys->init_actuator(2, 1, actuator_offsets[2]);
    sys->init_actuator(3, 1, actuator_offsets[3]);
    sys->init_actuator(4,-1, actuator_offsets[4]);
    sys->init_actuator(5,-1, actuator_offsets[5]);
    sys->init_actuator(6,-1, actuator_offsets[6]);
    sys->init_actuator(7,-1, actuator_offsets[7]);
    std::cout << "done!" << std::endl;

    for(i=0;i<4;i++)
        sys->set_torque_limit(i,TORQUE_LIMIT_WRIST,PEAK_TORQUE_WRIST);
    sys->set_torque_limit(mGROSS,TORQUE_LIMIT_GROSS,PEAK_TORQUE_GROSS);
    for(i=4;i<num_actuators-1;i++)
        sys->set_torque_limit(i,MAX_VOLTAGE_BASE, MAX_VOLTAGE_BASE);

    // create controllers
    double _proportional[num_actuators] = { WRIST1_P_PITCH, WRIST1_P_YAW1, WRIST1_P_YAW2, WRIST1_P_GROSS,
                                            BASE1_LEFT_P/M_COUNT, BASE1_REAR_P/M_COUNT, BASE1_RIGHT_P/M_COUNT, 10.0 };
    double _integral[num_actuators]     = { WRIST1_I_PITCH, WRIST1_I_YAW1, WRIST1_I_YAW2, WRIST1_I_GROSS,
                                            BASE1_LEFT_I/M_COUNT, BASE1_REAR_I/M_COUNT, BASE1_RIGHT_I/M_COUNT, 0.0 };
    double _derivative[num_actuators]   = { WRIST1_D_PITCH, WRIST1_D_YAW1, WRIST1_D_YAW2, WRIST1_D_GROSS,
                                            BASE1_LEFT_D/M_COUNT, BASE1_REAR_D/M_COUNT, BASE1_RIGHT_D/M_COUNT, 0.0 };

    if(MODE1 == PID){
        boost::shared_ptr<Controller> controllers(new Controller_pid(slave_num, num_actuators, num_actuators, _proportional, _integral, _derivative));
        return new Slave_accelera(slave_num, controllers, sys);
        }
    if(MODE1 == LQR){
        boost::shared_ptr<Controller_lqr> controllers(new Controller_lqr(slave_num,num_sensors,num_actuators,"lqr_feedback.log","lqr_targets.log"));
        return new Slave_accelera(slave_num, controllers, sys);
        }
    if(MODE1 == PID_AND_LQR){
        boost::shared_ptr<Controller_lqr_and_pid> controllers(new Controller_lqr_and_pid(slave_num,num_sensors,num_actuators,"lqr_feedback_slave_1.log","lqr_targets_slave_1.log",_proportional,_integral,_derivative));
        return new Slave_accelera(slave_num, controllers, sys);
        }
    if(MODE1 == VISUAL){
        boost::shared_ptr<Controller_visual> controllers(new Controller_visual(slave_num,num_sensors,num_actuators,"lqr_feedback_slave_1.log","lqr_targets_slave_1.log",_proportional,_integral,_derivative,NUMSERVOS1,SERVODURATION1));
        return new Slave_accelera(slave_num, controllers, sys);
    }
    if(MODE1 == VISUAL_SERVO_ONLY){
        boost::shared_ptr<Controller_visual_servo> controllers(new Controller_visual_servo(slave_num,num_sensors,num_actuators,_proportional,_integral,_derivative,NUMSERVOS1,SERVODURATION1));
        return new Slave_accelera(slave_num, controllers, sys);
    }
    if(MODE1 == VISUAL_MULTI_PLANNER_KNOTTIE){
        boost::shared_ptr<Controller_visual_knottie> controllers(
        new Controller_visual_knottie(slave_num,num_sensors,num_actuators,"lqr_feedback_slave_1.log","lqr_targets_slave_1.log",_proportional,_integral,_derivative));
        return new Slave_accelera(slave_num, controllers, sys);
    }
    else{
        debugprintf("Invalid Mode Defined for Slave 1.");
        assert(false);
    }

}   

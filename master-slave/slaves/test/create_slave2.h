#include "shared.h"
#include "imports.h"

    
Slave_mark1* CreateSlave2()
{
    short i;

    // choose sensors
    Sensor** sensors                = new Sensor*[num_sensors];
    sensors[0] = new MiranovaSensor(0, R_COUNT, 0,  1,1);
    sensors[1] = new MiranovaSensor(1, R_COUNT, 0,  1,1);
    sensors[2] = new MiranovaSensor(2, R_COUNT, 0, -1,1);
    sensors[3] = new MiranovaSensor(3, R_COUNT, 0, -1,1);
    sensors[4] = new MiranovaSensor(4, M_COUNT, 0, -1,1);
    sensors[5] = new MiranovaSensor(5, M_COUNT, 0, -1,1);
    sensors[6] = new MiranovaSensor(6, M_COUNT, 0, -1,1);
    sensors[7] = new DummySensor(0, 1, 0, 1);
    
    // choose actuators
    Actuator** actuators            = new Actuator*[num_sensors];
    double actuator_offsets[num_actuators] = { -0.05, -0.05, 0.1, 0, 0, 0, 0, 0 };
    actuators[0] = new QuatechDACActuator(0,  1, actuator_offsets[0],1);
    actuators[1] = new QuatechDACActuator(3,  1, actuator_offsets[1],1);
    actuators[2] = new QuatechDACActuator(5, -1, actuator_offsets[2],1);
    actuators[3] = new QuatechDACActuator(6, -1, actuator_offsets[3],1);
    actuators[4] = new QuatechDACActuator(1, -1, actuator_offsets[4],1);
    actuators[5] = new QuatechDACActuator(4, -1, actuator_offsets[5],1);
    actuators[6] = new QuatechDACActuator(7, -1, actuator_offsets[6],1);
    actuators[7] = new QuatechDualValveActuator(new QuatechDigitalActuator(0, 0x20), new QuatechDigitalActuator(0, 0x04));
    for(i=0;i<3;i++)
        actuators[i]->setLimit(-2.0,2.0);
    for(i=3;i<7;i++)
        actuators[i]->setLimit(-4.0,4.0);

    // create controllers
    double _proportional[num_actuators] = { 4.0, 4.0, 4.0, 6.0, 2.0, 2.0, 2.0, 1.0 };
    double _integral[num_actuators]     = { 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0 };
    double _derivative[num_actuators]   = { 0.0, 0.0, 0.0,-5.0,-2.0,-2.0,-2.0, 0.0 };
    Controller** controllers = new Controller*[2];
    controllers[0] = new Controller_pid(2, num_actuators-1, num_actuators-1, _proportional, _integral, _derivative);
    controllers[1] = new Controller_dummy(2,1,1);
    
    // create Slave
    boost::shared_ptr<Controller> contr(new Controller_generic(2, 2, num_actuators, controllers));
    boost::shared_ptr<System> sys(new System_generic(2, actuators, sensors));
    return new Slave_mark1(2, contr, sys);
}

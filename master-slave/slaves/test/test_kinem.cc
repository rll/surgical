#include "shared.h"
#include "create_slave1.h"
#include "create_slave1_accelera.h"
#include "create_slave2.h"

int main(int argc, char** argv) {
    Slave* slave = CreateSlave1_accelera();

    double examine[num_dof] = { 0.0, 0.0, 0.0, 0.0, 129.30, -233.26, 0.0 };
    double motors[num_actuators];
    double point[num_dof];

    slave->point_to_motors(examine, motors);
    slave->motors_to_point(motors, point);

    printf("Original Point:");
    for(int i=0;i<num_dof;i++) {
        printf("[%d] %f,", i, examine[i]);
    }
    printf("\n");

    printf("Motors:");
    for(int i=0;i<num_actuators;i++) {
        printf("[%d] %f,", i, motors[i]);
    }
    printf("\n");

    printf("Reverted:");
    for(int i=0;i<num_dof;i++) {
        printf("[%d] %f,", i, point[i]);
    }
    printf("\n");

    delete slave;
    return 0;
}

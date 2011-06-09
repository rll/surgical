/* test_actuators.cpp
 * this file is used to test sending RAW values to the motors
 * WARNING : cables should probably be disconnected before using this file.
 * If you want to move the slave, compile test_setpoints.cpp, which has slave-specific offsets and control variables
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "imports.h"
#define SLAVE_NUM 0

int main(int argc, char** argv) {
    boost::shared_ptr<Galil>        g(new Galil(SLAVE1_IP));
    boost::shared_ptr<System>       sys(new System_accelera(SLAVE_NUM,g));

    int i;
    for(i=0;i<num_actuators;i++)
        sys->init_actuator(i, 1, 0);    //Initialize Actuators
    for(i=0;i<4;i++)
        sys->set_limit(i,-1*MAX_VOLTAGE_WRIST,MAX_VOLTAGE_WRIST);
    for(i=4;i<num_actuators;i++)
        sys->set_limit(i,-1*MAX_VOLTAGE_BASE,MAX_VOLTAGE_BASE);

 	char c = ' ';
    double voltages[num_actuators];
    for(i=0;i<num_actuators;i++){
        voltages[i] = 0.0;
    }
	
    while(c != EOF) {
		printf("ch %d = %f volts\n", i,voltages[i]);
		c = getchar();
		if (c >= '0' && c <= '7') {
			int channel = c - '0';
			i = channel;
		} else if (c >= 'a' && c <= 'f') {
		    int channel = c - 'a' + 10;
		    i = channel;
		} else if (c == '+') {
            voltages[i] += 0.05;
		} else if (c == '-') {
            voltages[i] -= 0.05;
		} else if (c == 'r' || c == 'R') {
            voltages[i] = 0.0;
		}
        sys->queue_voltages(voltages);
        sys->apply_voltages();
	}
	return 0;
}

// miranova.cc

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <iostream>

#include "shared.h"
#include "imports.h"
#include "util.h"

#define MIRANOVA 0
#define ACCELERA 1
#define IP_ADDR "192.168.1.105 UDP"

#define TYPE ACCELERA
#define SLAVE_NUMBER 1

using namespace std;

int main() {
    try {
	int status = 0;
	int i = 0;
	long val = 0L;
    Sensor* sensors[num_sensors];
    boost::shared_ptr<Galil> g;
    System* sys;

    if (TYPE == ACCELERA) {
        g.reset(new Galil(IP_ADDR));
        g->command("KP*=0");    // Disable onboard PID
        g->command("KI*=0");
        g->command("KD*=0");
        g->command("OF*=0");    // All voltage offsets are off
        g->command("DP*=0");    // Here is 0 for all encoders
        sys = new System_accelera(SLAVE_NUMBER, g);
    }

	for(i=0; i<num_sensors; i++) {
        if (TYPE == MIRANOVA) {
            sensors[i] = new MiranovaSensor(i, 1, 0, 1, SLAVE_NUMBER);
        } else if (TYPE == ACCELERA) {
            sys->init_encoder(i, 1, 1, 0);
        }
    }
	int n = 0;
	while(1) {
		usleep(10);
		if(n % 10 != 0) 
            continue;
	    for(i=0; i<num_actuators; i++) {
            if (TYPE == MIRANOVA) {
			    printf("[%d]: %f\t", i, sensors[i]->Input());
            } else {
                if (i == 5)
                    printf("[%d]: %f\t", i, sys->read_encoder(i));
            }
		}
        sys->reset_all();
		printf("\n");
	}
    } catch (std::string s) {
        cout << s << endl;
    } catch (...) { }
	return 0;
}

#include "imports.h"
#include "create_slave1.h"
#include "master.h"
#include <signal.h>
#include <iostream>
#define MASTERS     true
#define IN_PORT     9000
#define OUT_PORT    9001
#define IP_ADDR     MASTERS_IP

Slave* slave;
Master* master;
bool first_run = true;

using namespace std;

/** Safely kill the slaves */
void terminate(int signal) {
    cout << "terminating..." << endl;
    for(int i = 0; i < num_actuators; ++i) {
		slave->_actuators[i]->Reset();
    }
    cout << "done" << endl;
    exit(0);
}

int main(int argc, char** argv) {
	char ch, raw_input;
	int channel = 0, i;
	double kp, kd, ki, max_error, id_bound;
	double setpoints[7]  = { 0.0, 0.0, 0.0, 0.0, 101.13, -173.7, 0.0 };  // pitch, roll, gross, x, y, z, grip
    double delta = 0.005;
	
	signal(SIGTERM, terminate);
    signal(SIGINT, terminate);
    signal(SIGSEGV, terminate);

	slave = CreateSlave1();
    slave->init_setpoints(setpoints[0], setpoints[1], setpoints[2], setpoints[3], setpoints[4], setpoints[5], setpoints[6]);
    master = new Master(1, IN_PORT, OUT_PORT, IP_ADDR);

    timeval now, then;
    gettimeofday(&then, NULL);
    gettimeofday(&now, NULL);

	while(raw_input != EOF) {
		kp = (dynamic_cast<PIDController*> (slave->_controllers[channel]))->_kp;
		kd = (dynamic_cast<PIDController*> (slave->_controllers[channel]))->_kd;
		ki = (dynamic_cast<PIDController*> (slave->_controllers[channel]))->_ki;
        //max_error = (dynamic_cast<PIDController*> (slave->_controllers[channel]))->_max_error;
        //id_bound = (dynamic_cast<PIDController*> (slave->_controllers[channel]))->_id_boundary;

		slave->move_to(setpoints);

        // Timed prints
        double diff = (((double) now.tv_sec) * 1000000 + now.tv_usec) - (((double) then.tv_sec) * 1000000 + then.tv_usec);
        if (diff > 300000) {
            printf("\nkp: %f, kd: %f, ki %f, max_err %f, id_bound %f\n", kp, kd, ki, max_error, id_bound);
            double goal_pos[num_actuators];
            slave->point_to_motors(setpoints, goal_pos);
            printf("goal:\t");
            for(i=0;i<7;i++)
                printf("[%d] %f, ", i, setpoints[i]);
            printf("\n");
            printf("error:\t");
            for(i=0;i<7;i++)
                printf("[%d] %f, ", i, goal_pos[i]-slave->_sensors[i]->Input());
            printf("\n");
            printf("delta: %f\n", delta);

            gettimeofday(&then, NULL);
        }
        gettimeofday(&now, NULL);

		if(_kbhit()) {
			raw_input = getchar();
			if(raw_input >= 48 && raw_input <= 57) {
				channel = raw_input - 48;
                continue;
			} else {
				ch = (char)raw_input;
				switch(ch) {
					case 'q':
						kp += delta;
						break;
					case 'a':
						kp -= delta;
						break;
					case 'w':
						kd += delta;
						break;
					case 's':
						kd -= delta;
						break;
					case 'e':
						ki += delta;
						break;
					case 'd':
						ki -= delta;
						break;
					case 'r':
                        max_error += delta;
						break;
					case 'f':
                        max_error -= delta;
						break;
					case 't':
                        id_bound += delta;
						break;
					case 'g':
                        id_bound -= delta;
						break;
					case 'y':
                        delta += 0.005;
						break;
					case 'h':
                        delta -= 0.005;
						break;
					case 'u':
                        delta *= 10;
						break;
					case 'j':
                        delta *= 0.1;
						break;
					case 'i':
						break;
					case 'k':
						break;
					default:
						cout << "invalid option" << endl;
						break;
				}
				
			}
		}
        vector<double> params;
        params.push_back(kp);
        params.push_back(ki);
        params.push_back(kd);
        //params.push_back(max_error);
        //params.push_back(id_bound);
        slave->_controllers[channel]->SetParams(params);
        if (MASTERS) {
            if (master->_accepted) {
                if (first_run) {
                    master->create_offsets(
                            -1*(setpoints[3])/(RADIUS_HEIGHT_RATIO*CONE_SPHERE_BOUND)*MAX_XY,
                            (setpoints[4])/(RADIUS_HEIGHT_RATIO*CONE_SPHERE_BOUND)*MAX_XY,
                            -1*((setpoints[5])/CONE_SPHERE_BOUND)*MAX_Z,
                            0,
                            0,
                            0);
                    first_run = false;
                }
                vector<double> position = master->get_destination();
                for(i=0;i<7;i++) {
                    setpoints[i] = position.at(i);
                }
                double safe[7];
                slave->nearest_legal_point(setpoints, safe);
                master->respond(setpoints, safe);

                slave->move_to(setpoints);
            } else {
                master->accept();
            }
        }
	}
	
	return 0;
}

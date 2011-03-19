/** A quick and dirty program to map out points in 
 * xyz space for slave 2.  This app is not safe for
 * general consumption */

#include "imports.h"
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <sys/io.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include "create_slave1.h"
#include "create_slave1_accelera.h"
#include "create_slave2.h"

#define QUICKHOME   false

using namespace std;

Slave_mark1* slave;

void terminate(int signal);

int main(int argc, char** argv) {
    signal(SIGTERM, terminate);
    signal(SIGINT, terminate);
    signal(SIGSEGV, terminate);

    ofstream record("voltage_mapping.txt");

    slave = CreateSlave1_accelera();
    if(QUICKHOME) {
        std::map<int,double> slave1_pts;
        slave1_pts[iPITCH] = M_PI;
        slave1_pts[iTILT]  = 0;
        slave1_pts[iGROSS] = 0;
        slave1_pts[iX] = 0;
        slave1_pts[iY] = 129.30;
        slave1_pts[iZ] = -233.26;
        slave1_pts[iGRIP] = 0;
        slave->init_setpoints(slave1_pts);
    } else {
        slave->home();
        cout << "Slave is homed" << endl;
    }

    int spacing = 4;
    int x_start = 300, y_start = 300, z_start = 300;
    int x_max = 400,  y_max = 400,  z_max = 400;
    double x=x_start,y=y_start,z=z_start;
    int index[3];
    double motor_positions[num_actuators];

    while(x < x_max) {
        index[0] = x;
        while(y < y_max) {
            index[1] = y;
            while (z < z_max) {
                index[2] = z;
                cout << "Considering (" << x << "," << y << "," << z << ")" << endl;

                double base_motors[3];
                base_motors[0] = x;
                base_motors[1] = y;
                base_motors[2] = z;

                for(int i=0;i<3;i++) {
                    motor_positions[i+4] = base_motors[i];
                }

                // Check legality of position
                double tip_position[7];
                double safe[7];
                bool safe_flag = true;
                // First, grab elbow position.  Is it too far from trocar?
                slave->motors_to_point(motor_positions, tip_position, true);
                int magnitude = 0;
                for(int i=0;i<3;i++) {
                    magnitude += tip_position[i]*tip_position[i];
                }
                magnitude = sqrt(magnitude);
                slave->motors_to_point(motor_positions, tip_position);
                if (magnitude > slave->stick_length - ARM_MIN) {
                    cout << "Point is out of trocar" << endl;
                    safe_flag = false;
                }
                // Next, make sure tip position is within safety
                slave->nearest_legal_point(tip_position, safe);
                for(int i=0;i<7;i++) {
                    if( fabs(tip_position[i] - safe[i]) > 0.001 ) {
                        cout << "Tip is not in legal pose: ";
                        cout << tip_position[3] << " ";
                        cout << tip_position[4] << " ";
                        cout << tip_position[5] << " ";
                        cout << "vs " << safe[3] << " ";
                        cout << safe[4] << " ";
                        cout << safe[5] << " ";
                        cout << endl;
                        safe_flag = false;
                        break;
                    }
                }

                // If it's safe, move there
                if(safe_flag) {
                    cout << "This position is safe.  Executing" << endl;
                    // Wait 2 sec until we arrive at new point
                    double time_till_record = 2*1000000;
                    while(time_till_record > 0) {
                        time_till_record -= 1000;
                        slave->motors_to(motor_positions);
                        usleep(1000);
                    }

                    // record 5 values in 100ms steps
                    int left_to_record = 5;
                    while(left_to_record > 0) {
                        cout << "Recording values left: " << left_to_record << endl;
                        int wait_time = 100000;
                        while (wait_time > 0) {
                            slave->motors_to(motor_positions);
                            usleep(1000);
                            wait_time -= 1000;
                        }
                        double motor_pos[num_actuators];
                        slave->current_position(motor_pos);
                        for(int i=0;i<3;i++) {
                            record << motor_pos[4+i] << " ";
                        }
                        for(int i=0;i<3;i++) {
                            double* input = slave->_sys->queued_voltages;
                            record << input[i+4] << " ";
                            cout << input[i+4] << " ";
                        }
                        record << endl;
                        cout << endl;
                        left_to_record -= 1;
                    }
                }

                z+=spacing;
            }
            z=z_start;
            y+=spacing;
        }
        y=y_start;
        x+=spacing;
    }
    x=x_start;

    return 0;
}

/** Safely kill the slaves */
void terminate(int signal) {
    cout << "terminating..." << endl;
	slave->_sys->reset_all();
    cout << "done" << endl;
    exit(0);
}

#include "shared.h"
#include "create_slave1.h"
#include "create_slave1_accelera.h"
#include "create_slave2.h"
#include "create_slave2_accelera.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <vector>

using namespace std;

Slave* s;

void terminate(int n) {
    s->_sys->reset_all();
    printf("DONE\n");
    exit(0);
    return;
}

int main(int argc, char** argv) {
    signal(SIGTERM, terminate);
    signal(SIGINT, terminate);
	int i;
	
    s = CreateSlave2_accelera();

    map<int,double> slave_pts;
    slave_pts[iPITCH] = 0;
    slave_pts[iTILT]  = 0;
    slave_pts[iGROSS] = 0;
    slave_pts[iX] = 0;
    slave_pts[iY] = 89.8660853;
    slave_pts[iZ] = -184.252779;
    slave_pts[iGRIP] = 0;
    s->init_setpoints(slave_pts);

    s->_sys->reset_all();

    double loc[num_dof];
	while(1) {
        s->current_pose(loc, true);
        printf("loc: ");
        for(int i=0;i<num_actuators;i++) {
            printf("[%d] %f, ",i, loc[i]);
        }
        printf("\n");
	}
	return 0;
}


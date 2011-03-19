#include "shared.h"
#include "imports.h"
#include "create_slave1.h"
#include "create_slave1_accelera.h"
#include "create_slave2.h"
#include "create_slave2_accelera.h"
#include "master.h"
#include "Socket.h"

enum{NORMALHOME,QUICKHOME,MANUALHOME};

#define IN_PORT     9000
#define OUT_PORT    9001
#define IN_PORT2    9002
#define OUT_PORT2   9003

enum{SLAVE1,SLAVE2,BOTH};

#define MODE        BOTH

#define USE_SLAVE_1 MODE==SLAVE1 || MODE==BOTH
#define USE_SLAVE_2 MODE==SLAVE2 || MODE==BOTH

#define HOMETYPE    NORMALHOME


#define MASTERS     true


#define IGNORE_KEYS false

using namespace std;
Slave* slaves[num_slaves];
Master* masters[num_slaves];
void terminate(int signal);
void print_slave_info(double setpoints[num_slaves][num_dof]);

int main(int argc, char** argv) {
    try{
    unsigned int i;
    int first_run = 1;
    double setpoints[num_slaves][num_dof];
    signal(SIGTERM, terminate);
    signal(SIGINT, terminate);
    signal(SIGSEGV, terminate);
    
    // Create slaves without resetting points
    slaves[0] = CreateSlave1_accelera();
    slaves[1] = CreateSlave2_accelera();
    slaves[0]->_contr->set_other_controller(slaves[1]->_contr);
    slaves[1]->_contr->set_other_controller(slaves[0]->_contr);

    if (HOMETYPE == NORMALHOME) {
        if(USE_SLAVE_1)
            slaves[0]->home();
        if(USE_SLAVE_2)
            slaves[1]->home();
    } else if (HOMETYPE == MANUALHOME) {
        map<int,double> slave1_pts, slave2_pts;

        slave1_pts[iPITCH] = M_PI/2;
        slave1_pts[iTILT]  = M_PI/2;
        slave1_pts[iGROSS] = (17/6)*M_PI/2;
        slave1_pts[iX] = 0;
        slave1_pts[iY] = 129.30;
        slave1_pts[iZ] = -233.26;
        slave1_pts[iGRIP] = GRIP_ON;
        slaves[0]->init_setpoints(slave1_pts);

        slave2_pts[iPITCH] = 0;
        slave2_pts[iTILT]  = 0;
        slave2_pts[iGROSS] = 0;
        slave2_pts[iX] = 0;
        slave2_pts[iY] =  89.8660853;
        slave2_pts[iZ] = -184.252779;
        slave2_pts[iGRIP] = GRIP_ON;
        slaves[1]->init_setpoints(slave2_pts);

    } else if (HOMETYPE == QUICKHOME) {
        map<int,double> slave1_pts, slave2_pts;

        slave1_pts[iPITCH] = M_PI/2;
        slave1_pts[iTILT]  = M_PI/2;
        slave1_pts[iGROSS] = M_PI;
        //slave1_pts[iX] = -1.27;
        //slave1_pts[iY] = 63.37;
        //slave1_pts[iZ] = -52.65;
        slave1_pts[iX] = 0;
        slave1_pts[iY] = 129.30;
        slave1_pts[iZ] = -233.26;
        slave1_pts[iGRIP] = GRIP_ON;
        slaves[0]->init_setpoints(slave1_pts);


        slave2_pts[iPITCH] = M_PI/2;
        slave2_pts[iTILT]  = M_PI/2;
        slave2_pts[iGROSS] = M_PI;
        slave2_pts[iX] = 4.93;
        slave2_pts[iY] =  55.13;
        slave2_pts[iZ] = -55.08;
        slave2_pts[iGRIP] = GRIP_ON;
        slaves[1]->init_setpoints(slave2_pts);
    } else {
        printf("Improper Homing Type defined.  Check top and try again\n");
        exit(0);
    }

    bool newRecording = true;
while(newRecording){
    // Obtain setpoints
    for(int j=0;j<num_slaves;j++){
        double pos[num_dof];
        slaves[j]->current_pose(pos);
        for(i=0;i<num_dof;i++){
            setpoints[j][i] = pos[i];
        }
    }

    // print out location
    bool warn = true;
    while (warn) {
        print_slave_info(setpoints);
        cout << "Are these positions sensable? (r to re-detect, y for yes, n for no)" << endl;
        char c = getchar();
        if (c == 'y') {         // Point is good, continue
            warn = false;
            //printf("Giving you a few seconds to hold slave in position");
            //usleep(1000000);
            ((Slave_accelera*) slaves[0])->reset_wrists();
            ((Slave_accelera*) slaves[1])->reset_wrists();
            for(int j=0;j<num_slaves;j++){
                double pos[num_dof];
                slaves[j]->current_pose(pos);
                for(i=0;i<num_dof;i++){
                    setpoints[j][i] = pos[i];
                }
            }
            printf("\n");
            
        } else if (c == 'n') {  // Point is bad, exit
            terminate(0);
        } else if (c == 'r') {  // Try getting location again
            for(int j=0;j<num_slaves;j++){
                double pos[num_dof];
                slaves[j]->current_pose(pos);
                for(i=0;i<num_dof;i++){
                    setpoints[j][i] = pos[i];
                }
            }
            printf("\n");
        }
    }
    

    // Masters init
    masters[0] = new Master(1, IN_PORT, OUT_PORT, MASTERS_IP);
    masters[1] = new Master(2, IN_PORT2, OUT_PORT2, MASTERS_IP);

    cout << "starting control loop" << endl;

    timeval now, then;
    gettimeofday(&then, NULL);
    gettimeofday(&now, NULL);

    //slaves[0]->start_logging();
    //slaves[1]->start_logging();

    while(1) {
        if(USE_SLAVE_1)
            slaves[0]->move_to(setpoints[0]);
        if(USE_SLAVE_2) 
        slaves[1]->move_to(setpoints[1]);
        
        // Timed prints
        double diff = timediff(now, then);
        if (diff > 300000 && !MASTERS) {
            print_slave_info(setpoints);
            gettimeofday(&then, NULL);
        }
        gettimeofday(&now, NULL);
        
        if(MASTERS) {
            if(!(masters[0]->accept() && masters[1]->accept())) {
               continue;
            }
            cout << "received connection" << endl;
            while(masters[0]->_accepted && masters[1]->_accepted) {
                if (first_run) {
                    double slave1_radius_height_ratio = dynamic_cast<Slave_mark1*>(slaves[0])->radius_height_ratio;
                    double slave1_cone_sphere_bound = dynamic_cast<Slave_mark1*>(slaves[0])->cone_sphere_bound;
                    double slave2_radius_height_ratio = dynamic_cast<Slave_mark1*>(slaves[1])->radius_height_ratio;
                    double slave2_cone_sphere_bound = dynamic_cast<Slave_mark1*>(slaves[1])->cone_sphere_bound;
                    //printf("Dynamic casting ok!\n");
                    masters[0]->create_offsets(
                            setpoints[0][0],
                            setpoints[0][1],
                            setpoints[0][2],
                            setpoints[0][3],
                            setpoints[0][4],
                            setpoints[0][5],
                            setpoints[0][6],
                            fabs(slave1_cone_sphere_bound*slave1_radius_height_ratio/(MAX_XY*0.9)));
                    //printf("master 1 configured\n");
                    masters[1]->create_offsets(
                            setpoints[1][0],
                            setpoints[1][1],
                            setpoints[1][2],
                            setpoints[1][3],
                            setpoints[1][4],
                            setpoints[1][5],
                            setpoints[1][6],
                            //fabs(RADIUS_HEIGHT_RATIO*CONE_SPHERE_BOUND/MAX_XY));
                            fabs(slave2_cone_sphere_bound*slave2_radius_height_ratio/(MAX_XY*0.9)));
                    //printf("master 2 configured\n");
                    if(USE_SLAVE_1)
                        slaves[0]->start_logging();
                    if(USE_SLAVE_2)
                        slaves[1]->start_logging();
                    first_run = false;
                }

                for(int j=0;j<num_slaves;j++){
                    //printf("For slave %d\n", j+1);
                    vector<double> position = masters[j]->get_destination();
                    for(i=0;i<num_dof;i++) {
                        setpoints[j][i] = position.at(i);
                    }
                    
                    //printf("\tobtained master position\n");
                    double safe[num_dof];
                    slaves[j]->nearest_legal_point(setpoints[j], safe);
                    //printf("\tobtained nearest legal point\n");
                    //masters[j]->respond(setpoints[j], safe);

                    // Timed prints
                    double diff = timediff(now, then);
                    if (diff > 300000) {
                        print_slave_info(setpoints);
                        gettimeofday(&then, NULL);
                    }
                    gettimeofday(&now, NULL);
                    if(j==0 && USE_SLAVE_1 || j==1 && USE_SLAVE_2)
                        slaves[j]->move_to(setpoints[j]);
                }
    	        if(_kbhit()) {
    	        	char c = (char)_getch();
                    if (c == 27) { // ESC key
                        terminate(0);
                    }
                }
            }
            if(USE_SLAVE_1)
                slaves[0]->stop_logging();
            if(USE_SLAVE_2)
                slaves[1]->stop_logging();
            first_run = true;
            break;
        } else {
    	    if(_kbhit()) {

    	    	double value;
    	    	char c = (char)_getch();
    	    	if(!IGNORE_KEYS || c == 27){
                    switch(c) {

                    case 'q':
                        setpoints[0][0] += M_PI/8;
                        setpoints[1][0] += M_PI/8;
                        break;
                    case 'a':
                        setpoints[0][0] -= M_PI/8;
                        setpoints[1][0] -= M_PI/8;
                        break;
                    case 'w':
                        setpoints[0][1] += M_PI/8;
                        setpoints[1][1] += M_PI/8;
                        break;
                    case 's':
                        setpoints[0][1] -= M_PI/8;
                        setpoints[1][1] -= M_PI/8;
                        break;
                    case 'e':
                        setpoints[0][2] += M_PI/8;
                        setpoints[1][2] += M_PI/8;
                        break;
                    case 'd':
                        setpoints[0][2] -= M_PI/8;
                        setpoints[1][2] -= M_PI/8;
                        break;
                    case 'r':
                        setpoints[0][3] += 2.5;
                        setpoints[1][3] += 2.5;
                        break;
                    case 'f':
                        setpoints[0][3] -= 2.5;
                        setpoints[1][3] -= 2.5;
                        break;
                    case 't':
                        setpoints[0][4] += 2.5;
                        setpoints[1][4] += 2.5;
                        break;
                    case 'g':
                        setpoints[0][4] -= 2.5;
                        setpoints[1][4] -= 2.5;
                        break;
                    case 'y':
                        setpoints[0][5] += 2.5;
                        setpoints[1][5] += 2.5;
                        break;
                    case 'h':
                        setpoints[0][5] -= 2.5;
                        setpoints[1][5] -= 2.5;
                        break;
                    case 'u':
                        setpoints[0][6] = GRIP_ON;
                        setpoints[1][6] = GRIP_ON;
                        break;
                    case 'j':
                        setpoints[0][6] = GRIP_OFF;
                        setpoints[1][6] = GRIP_OFF;
                        break;
                    case 27:
                        terminate(0);
                        break;
                    default:
                        break;
                    }
                }
    	    }
        }
    }
    newRecording = false;
    
    cout << "Would you like to record another trajectory? 'y' if yes." << endl;
    if (getchar()=='y'){
        newRecording = true;
        slaves[0]->_sys->reset_motors();
        slaves[1]->_sys->reset_motors();
        usleep(100000);   
    }
    else{
        terminate(0);
    }
    cout << "Press any key continue..." << endl;
    while(!_kbhit()){
        slaves[0]->_sys->reset_motors();
        slaves[1]->_sys->reset_motors();
    };
    _getch();
}
    } catch (string s) {
        cout << s << endl;
    }
    return 0;
}

/** Safely kill the slaves */
void terminate(int signal) {
    cout << "terminating..." << endl;
    double voltages[num_actuators];
    for(int s=0; s<num_slaves;s++) {
        slaves[s]->_sys->reset_all();
    }
    exit(0);
}

void print_slave_info(double setpoints[num_slaves][num_dof]){
    //printf("Grip set pos: %f\n",setpoints[0][iGRIP]);
    //return; //FIXME
    for(int j=0;j<num_slaves;j++) {
        if(j==0 && USE_SLAVE_1 || j==1 && USE_SLAVE_2){
            printf("set   %d: ", j+1);
            
            for(int i=0;i<num_dof;i++) {
                printf("[%d] %f, ", i, setpoints[j][i]);
            }
            
            printf("\nsetm  %d: ",j+1);
            double motors[num_actuators];
            slaves[j]->point_to_motors(setpoints[j],motors);
            for(int i=0;i<num_actuators;i++){
                printf("[%d] %f, ", i, motors[i]);
            }
            slaves[j]->print();
            
        }
    }
}

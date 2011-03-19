#include "shared.h"
#include "imports.h"
#include "create_slave1.h"
#include "create_slave1_accelera.h"
#include "create_slave2.h"
#include "create_slave2_accelera.h"

using namespace std;

#define QUICKHOME   false
#define MODIFY_DEMONSTRATION false

#define SLAVE1      0
#define SLAVE2      1
#define BOTH        2

#define MODE        BOTH

#define USE_SLAVE_1 MODE==SLAVE1 || MODE==BOTH
#define USE_SLAVE_2 MODE==SLAVE2 || MODE==BOTH
Slave* slaves[2];
void terminate(int signal);
void usage();
void print_slave_info();

int main(int argc, char** argv) {
    try {
    short i;
    string s;
    ifstream input;
    double wait_time = 0, this_time=0, last_time = 0;
    char null[1024];
    timeval now, previous;
    gettimeofday(&now, NULL);
    gettimeofday(&previous, NULL);

    if(!(argc >= 1)) {
        usage();
        exit(0);
    }
    else {
        input.open(argv[1]);
    }

    signal(SIGTERM, terminate);
    signal(SIGINT, terminate);
    signal(SIGSEGV, terminate);

    // prepare default positions
    double motor_positions[2][num_actuators];

    // Init slaves, assumed homed
    slaves[0] = CreateSlave1_accelera(); 
    slaves[1] = CreateSlave2_accelera();
    slaves[0]->_contr->set_other_controller(slaves[1]->_contr);
    slaves[1]->_contr->set_other_controller(slaves[0]->_contr);

    sleep(1);
    
    double gripper[2];
    gripper[0] = 0;
    gripper[1] = 0;
    double pitcher[2];
    pitcher[0] = 0;
    pitcher[1] = 0;
    double grosser[2];
    grosser[0] = 0;
    grosser[1] = 0;

    if (QUICKHOME) {
        cout << "QUICKHOMEing...";
        map<int,double> slave1_pts, slave2_pts;

        slave1_pts[iPITCH] = M_PI/2;
        slave1_pts[iTILT]  = M_PI/2;
        slave1_pts[iGROSS] = (17/6)*M_PI/2;
        slave1_pts[iX] = 0;
        slave1_pts[iY] = 129.30;
        slave1_pts[iZ] = -233.26;
        slave1_pts[iGRIP] = 0;
        slaves[0]->init_setpoints(slave1_pts);

        slave2_pts[iPITCH] = 0;
        slave2_pts[iTILT]  = 0;
        slave2_pts[iGROSS] = 0;
        slave2_pts[iX] = 0;
        slave2_pts[iY] =  89.8660853;
        slave2_pts[iZ] = -184.252779;
        slave2_pts[iGRIP] = 0;
        slaves[1]->init_setpoints(slave2_pts);
       
        
        cout << "done!" << endl;
    } else {
        if(USE_SLAVE_1)
            slaves[0]->home();
        if(USE_SLAVE_2)
            slaves[1]->home();
        
    }
    
    // print out location
    bool warn = true;
    while (warn) {
        print_slave_info();
        cout << "Are these positions sensable? (r to re-detect, y for yes, n for no)" << endl;
        char c = getchar();
        if (c == 'y') {         // Point is good, continue
            warn = false;
        } else if (c == 'n') {  // Point is bad, exit
            terminate(0);
        } else if (c == 'r') {  // Try getting location again
            for(int j=0;j<num_slaves;j++){
                double pos[num_dof];
                slaves[j]->current_pose(pos);
            }
            printf("\n");
        }
    }
    
    bool demonstrating = true;
    bool repeat = true;
    
    while(repeat){
    gripper[0] = 0;
    gripper[1] = 0;
    pitcher[0] = 0;
    pitcher[1] = 0;
    grosser[0] = 0;
    grosser[1] = 0;
    demonstrating = true;
    cout<< "Press any key to begin playback...";
    getchar(); 

    for(int j=0;j<2;j++){
        double pos[num_dof];
        slaves[j]->current_pose(pos);
        slaves[j]->point_to_motors(pos, motor_positions[j]);
    }
    printf("Beginning playback\n");
    

    // Prepare times
    getline(input, s);
    vector<double> parsed;

    for(i=0;i<num_actuators+2;i++) {    // slave_num + time + motor lengths
        double v = 0;
        input >> v;
        parsed.push_back(v);
    }
    // throw away the rest of the values
    input.getline(null, 1024);

    this_time = parsed[1];
    if(USE_SLAVE_1)
        slaves[0]->start_logging();
    if(USE_SLAVE_2)
        slaves[1]->start_logging();
    while(demonstrating) {
        
        if(_kbhit()) {
            char c = (char)_getch();
            if(MODIFY_DEMONSTRATION){
                switch(c){
                    case 'q':
                        gripper[0] += 1;
                        printf("Gripper changed to %f\n",gripper[0]);
                        break;
                    case 'a':
                        gripper[0] -= 1;
                        printf("Gripper changed to %f\n",gripper[0]);
                        break;
                    case 'w':
                        gripper[1] += 1;
                        printf("Gripper changed to %f\n",gripper[1]);
                        break;
                    case 's':
                        gripper[1] -= 1;
                        printf("Gripper changed to %f\n",gripper[1]);
                        break;
                    case 'e':
                        pitcher[0] += 1;
                            printf("Pitcher changed to %f\n",pitcher[0]);
                    break;
                    case 'd':
                        pitcher[0] -= 1;
                        printf("Pitcher changed to %f\n",pitcher[0]);
                        break;
                    case 'r':
                    pitcher[1] += 1;
                        printf("Pitcher changed to %f\n",pitcher[1]);
                        break;
                    case 'f':
                        pitcher[1] -= 1;
                        printf("Pitcher changed to %f\n",pitcher[1]);
                        break;
                    
                    case 't':
                        grosser[0] += 1;
                        printf("Grosser changed to %f\n",grosser[0]);
                        break;
                    case 'g':
                        grosser[0] -= 1;
                        printf("Grosser changed to %f\n",grosser[0]);
                        break;
                        
                    case 'y':
                        grosser[1] += 1;
                        printf("Grosser changed to %f\n",grosser[1]);
                        break;
                    case 'h':
                        grosser[1] -= 1;
                        printf("Grosser changed to %f\n",grosser[1]);
                        break;
                    
                    case 'p':
                        demonstrating = false;
                        slaves[0]->stop_logging();
                        slaves[1]->stop_logging();
                        slaves[0]->_sys->reset_all();
                        slaves[1]->_sys->reset_all(); 
                        break;      
                }      
            }
            else{
                switch(c){
                    case '1':
                        slaves[0]->log("%%BEGIN SLAVE 1 VISION%%\n");
                        slaves[1]->log("%%BEGIN SLAVE 1 VISION%%\n");
                        printf("Added flag to start slave 1's vision!\n");
                    break;
                    
                    case '2':
                        slaves[0]->log("%%END SLAVE 1 VISION%%\n");
                        slaves[1]->log("%%END SLAVE 1 VISION%%\n");
                        printf("Added flag to end slave 1's vision!\n");
                    break;
                    
                    case '3':
                        slaves[0]->log("%%BEGIN SLAVE 2 VISION%%\n");
                        slaves[1]->log("%%BEGIN SLAVE 2 VISION%%\n");
                        printf("Added flag to start slave 2's vision!\n");
                    break;
                        
                    case '4':
                        slaves[0]->log("%%END SLAVE 2 VISION%%\n");
                        slaves[1]->log("%%END SLAVE 2 VISION%%\n");
                        printf("Added flag to end slave 2's vision!\n");
                    break;        
                    
                    case '5':
                        slaves[0]->log("%%SLAVE 1 PAUSE%%\n");
                        slaves[1]->log("%%SLAVE 2 PAUSE%%\n");
                        printf("Added pause flags\n");
                    break;                                                                    
                    
                    default:
                        terminate(0);
                    break;
                }
            }
        }   
    
        for(int n=0;n<num_slaves;n++){
            motor_positions[n][0] = gripper[n];
            motor_positions[n][1] = pitcher[n];
            motor_positions[n][2] = grosser[n];
        }
        if(demonstrating){
            if(USE_SLAVE_1)
                slaves[0]->motors_to(motor_positions[0]);
            if(USE_SLAVE_2)
                slaves[1]->motors_to(motor_positions[1]);
        }

        previous = now;
        gettimeofday(&now, NULL);
        wait_time -= timediff(now, previous);
        if(wait_time <= 0) {
            double slave_num;
            input >> slave_num;
            if (!input.eof()) {
                // slave_num exists, so we can parse the rest
                vector<double> parsed;
                parsed.push_back(slave_num);
                for(i=0;i<num_actuators+1;i++) {    // time + motor lengths
                    double v = 0;
                    input >> v;
                    parsed.push_back(v);
                }
                
                // throw away the rest of the values
                input.getline(null, 1024);

                // print out and copy position into motor_positions
                //printf("for slave[%d]: ", (int)parsed.at(0));
                for(i=0;i<num_actuators;i++){
                    double goal = parsed.at(i+2);
                    //printf("[%d] %f, ", i, goal);
                    motor_positions[((int)parsed.at(0)-1)][i] = goal;
                }

                // calculate time till next read
                last_time = this_time;
                this_time = parsed.at(1);
                wait_time = this_time - last_time;
                //printf("\n");
            } else {
                input.close();
            }
        }
    }
    }

    } catch (std::string s) {
        cout << s << endl;
    }
}

/** Safely kill the slaves */
void terminate(int signal) {
    cout << "terminating..." << endl;
    for(int j = 0; j < 2; j++) {
        slaves[j]->_sys->reset_all();
    }
    cout << "done" << endl;
    exit(0);
}

void usage() {
    printf("Usage:  replay <combined logfile>\n");
}

void print_slave_info(){
    for(int j=0;j<num_slaves;j++) {
        if(j==0 && USE_SLAVE_1 || j==1 && USE_SLAVE_2){
            slaves[j]->print();
            
        }
    }
}


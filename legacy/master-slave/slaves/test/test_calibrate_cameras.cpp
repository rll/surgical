/* Sends relevant calibration information to Vision */
#include "shared.h"
#include "imports.h"
#include "create_slave1.h"
#include "create_slave1_accelera.h"
#include "create_slave2.h"
#include "create_slave2_accelera.h"
#include "UDPSocket.h"
#include "messaging.h"
#include "master.h"

enum{NORMALHOME,QUICKHOME,MANUALHOME};

#define IN_PORT     9000
#define OUT_PORT    9001
#define IN_PORT2    9002
#define OUT_PORT2   9003


enum{SLAVE1,SLAVE2,BOTH};

#define MODE        SLAVE2

#define USE_SLAVE_1 MODE==SLAVE1 || MODE==BOTH
#define USE_SLAVE_2 MODE==SLAVE2 || MODE==BOTH


#define USING_MASTERS     true
#define IGNORE_KEYS false

using namespace std;
Slave* slaves[num_slaves];
Master* masters[num_slaves];
void terminate(int signal);
void print_slave_info(double setpoints[num_slaves][num_dof]);
void pause_slaves();
void unpause_slaves();
bool paused;
bool accepted;
bool has_accepted();
bool first_run = true;
double setpoints[num_slaves][num_dof];
UDPSocket   _incoming, _outgoing;
timeval last_sent_request;
int main(int argc, char** argv) {
    paused = false;
    accepted = false;
    _incoming       = UDPSocket();
    _outgoing       = UDPSocket();
    gettimeofday(&last_sent_request, NULL);
    double last_position[num_slaves][num_dof];
    
    try{
    unsigned int i;
    
    
    signal(SIGTERM, terminate);
    signal(SIGINT, terminate);
    signal(SIGSEGV, terminate);
    
    // Create slaves without resetting points
    slaves[0] = CreateSlave1_accelera();
    slaves[1] = CreateSlave2_accelera();

    if(USE_SLAVE_1)
        slaves[0]->home();
    if(USE_SLAVE_2)
        slaves[1]->home();
    bool controlLoop = true;
    
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
    // Vision init
    if(!_incoming.create())
        std::cout << "error creating server socket" << std::endl;
    if(!_incoming.bind(CONTROLLER_IN_PORT))
        std::cout << "error binding to port " << CONTROLLER_IN_PORT << std::endl;

    if(!_outgoing.create())
        std::cout << "error creating server socket" << std::endl;
    if(!_outgoing.bind(VISION_IN_PORT))
        std::cout << "error binding to port " << VISION_IN_PORT << std::endl;

    _incoming.set_non_blocking(true);
    _incoming.set_timeout(0);             // using recv()    
    _outgoing.set_non_blocking(true);
    _outgoing.set_timeout(0);
    _outgoing.setDestination(VISION_IP, VISION_IN_PORT);


    cout << "starting control loop" << endl;
while(controlLoop){

    


    timeval now, then;
    gettimeofday(&then, NULL);
    gettimeofday(&now, NULL);
    


    while(1) {
        if(USE_SLAVE_1)
            slaves[0]->move_to(setpoints[0]);
        if(USE_SLAVE_2) 
            slaves[1]->move_to(setpoints[1]);
        
        // Timed prints
        double diff = timediff(now, then);
        /*if (diff > 300000) {
            print_slave_info(setpoints);
            gettimeofday(&then, NULL);
        }
        */
        gettimeofday(&now, NULL);

        if(paused){
            if(!has_accepted()){
                /*
                double pos[num_slaves][num_dof];
                for(int i = 0; i < num_slaves; i++){
                    slaves[i]->current_pose(pos[i]);
                }
                if(timediff(now, last_sent_request) < 1000000){
                    Messaging::send_message(_outgoing, Messaging::CALIBRATION_REQUEST, pos[0][iX],pos[0][iY],pos[0][iZ],pos[1][iX],pos[1][iY],pos[1][iZ]);
                    gettimeofday(&last_sent_request, NULL);
                }
                */
                usleep(1000);
            }
            else{
                vector<double> position;
                if(USE_SLAVE_1)
                    position = masters[0]->get_destination();
                else
                    position = masters[1]->get_destination();
                if(position.at(iGRIP) == GRIP_ON){
                    unpause_slaves();
                }
                else{
                    for(int j = 0; j < num_slaves; j++){
                        if(j==0 && USE_SLAVE_1 || j==1 && USE_SLAVE_2)
                        slaves[j]->move_to(setpoints[j]);
                    }  
                    usleep(5000);
                }
            }
        }
        else if(USING_MASTERS) {
            if(!(masters[0]->accept() && masters[1]->accept())) {
               continue;
            }
            cout << "received connection" << endl;
            while(masters[0]->_accepted && masters[1]->_accepted && !paused) {
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
                    if( ((j==0 && USE_SLAVE_1) || (j==1 && USE_SLAVE_2)) && position.at(iGRIP) == GRIP_OFF)
                        pause_slaves();
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
                    if (c=='p'){
                        pause_slaves();    
                    }
                    if (c=='u'){
                        unpause_slaves();
                    }
                }
            }
            first_run = true;
            break;
        }
     if(_kbhit()) {

    	    	double value;
    	    	char c = (char)_getch();
    	    	if(!IGNORE_KEYS || c == 27){
                    switch(c) {
                    
                    case 'p':
                        pause_slaves();
                        break;
                        
                    case 'u':
                        unpause_slaves();
                        break;
                   
                    case 27:
                        terminate(0);

                    default:
                        break;
                    }
                }
    	    }
    }
}
    controlLoop = false;
    
    cout << "Would you like to record another trajectory? 'y' if yes." << endl;
    if (getchar()=='y'){
        controlLoop = true;
        for(int i = 0; i < 20; i++){
            slaves[0]->_sys->reset_all();
            slaves[1]->_sys->reset_all();
            usleep(1000);
        }   
    }
    cout << "Press any key continue..." << endl;
    getchar();
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
    cout << "done" << endl;
    exit(0);
}

void print_slave_info(double setpoints[num_slaves][num_dof]){
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

void pause_slaves(){
    printf("Pausing slaves.");
    paused = true;
    accepted = false;
    double pos[num_slaves][num_dof];
    for(int i = 0; i < num_slaves; i++){
       slaves[i]->current_pose(pos[i]);
    }
    Messaging::send_message(_outgoing, Messaging::CALIBRATION_REQUEST, pos[0][iX],pos[0][iY],pos[0][iZ],pos[1][iX],pos[1][iY],pos[1][iZ]);

}

void unpause_slaves(){
    printf("UNPAUSING slaves in");
    for(int i = 5; i > 0; i--){
        printf("...%d",i);
        //usleep(1000000);
    }
    printf("...DONE\n");
    paused = false;
    accepted = false;
    //double positions[num_slaves][num_dof];
    //slaves[0]->current_pose(positions[0]);
    //slaves[1]->current_pose(positions[1]);
    //for(int i = 0; i < num_slaves; i++){
    //    for(int j = 0; j < num_dof; j++){
    //        setpoints[i][j] = positions[i][j];
    //    }
    //}
    delete masters[0];
    delete masters[1];
    masters[0] = new Master(1, IN_PORT, OUT_PORT, MASTERS_IP);
    masters[1] = new Master(2, IN_PORT2, OUT_PORT2, MASTERS_IP);
    first_run = true;
        
     
}

bool has_accepted(){
    return true; //FIXME
    if(!accepted){
        if(Messaging::receive_message(_incoming, Messaging::RECEIVED)){
            accepted = true;
            printf("RECEIVED\n");
        }
    }
    return accepted;
}

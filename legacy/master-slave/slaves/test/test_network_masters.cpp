#include "messaging.h"
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <cstring>
#include <vector>
#include "matrix.h"

using namespace std;


int main(int argc, char** argv) {
    UDPSocket _incoming, _outgoing;
    _incoming = UDPSocket();
    _outgoing = UDPSocket();
    
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
    _outgoing.setDestination(CONTROLLER_IP, CONTROLLER_IN_PORT);
    
    //double a,b,c,d,e,f;
    //
    //Messaging::send_message(_outgoing, Messaging::CALIBRATION_REQUEST, 10.0,11.0,12.0,13.0,14.0,15.0);
    //while(!Messaging::receive_message(_incoming, Messaging::CALIBRATION_REQUEST, &a,&b,&c,&d,&e,&f)){
    //    printf("Waiting...\n");
    //
    //}
    //printf("Received:%f,%f,%f,%f,%f,%f\n",a,b,c,d,e,f);
    //
    char buf[256];
    sprintf(buf,"HELLO");
    printf("%s\n",buf);
    sprintf(buf,"GOODBYE");
    printf("%s\n",buf);
    sprintf(buf,"100%%");
    char buf2[50];
    strcpy(buf2,buf);
    char* buf3;
    buf3 = "DIDN'T WORK";
    buf3 = "WORKED";
    usleep(10000000);
    printf("%s\n",buf3);
        

}

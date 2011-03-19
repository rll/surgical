#include "visual_feedback.h"

Visual_feedback::Visual_feedback(int in_port, int out_port, const char* ip_addr){
    _incoming       = UDPSocket();
    _outgoing       = UDPSocket();
    _accepted       = false;
    
    // Initialize sockets
    if(!_incoming.create())
        std::cout << "error creating server socket" << std::endl;
    if(!_incoming.bind(in_port))
        std::cout << "error binding to port " << in_port << std::endl;

    if(!_outgoing.create())
        std::cout << "error creating server socket" << std::endl;
    if(!_outgoing.bind(out_port))
        std::cout << "error binding to port " << out_port << std::endl;

    _incoming.set_non_blocking(true);
    _incoming.set_timeout(0);             // using recv()

    _outgoing.set_non_blocking(true);
    _outgoing.set_timeout(0);
    _outgoing.setDestination(ip_addr , out_port);
}
    
Visual_feedback::~Visual_feedback(){
    if(&_incoming) delete &_incoming;
    if(&_outgoing) delete &_outgoing;
}

bool Visual_feedback::received(){
    return true;
    if(Messaging::receive_message(_incoming, Messaging::RECEIVED)){
        printf("Message Received\n");
        return true;
    }
    return false;
}

bool Visual_feedback::request_goal_position(){
    return Messaging::send_message(_outgoing, Messaging::THREAD_REQUEST);
}

bool Visual_feedback::request_current_position(){
    return Messaging::send_message(_outgoing, Messaging::POSITION_REQUEST);
}

bool Visual_feedback::get_goal_position(double* pos){
    double camX, camY, camZ;
    bool success = Messaging::receive_message(_incoming, Messaging::THREAD_RESPONSE, &camX, &camY, &camZ);
    //For now, no conversion between camera_pos and slave_pos -- handled by vision
    if(!success)
        return false;
    else{
        pos[iX] = camX;
        pos[iY] = camY;
        pos[iZ] = camZ;
        return true;
    }
}

bool Visual_feedback::get_current_position(double* pos){
    double camX, camY, camZ=0;
    bool success = Messaging::receive_message(_incoming, Messaging::POSITION_RESPONSE, &camX, &camY, &camZ);
    if(!success)
        return false;
    else{
        pos[iX] = camX;
        pos[iY] = camY;
        pos[iZ] = camZ;
        return success;
    }
}


#include "messaging.h"

#define RESEND_TIME 1000000

bool Messaging::send_message(UDPSocket& recipient, MessageType type, ...){
    va_list ap;
    va_start(ap, type);
    std::string msg = "";
    char buf[256];
    sprintf(buf,"HEAD %s:",message_info(type).flag);
    msg.append(buf);
    for(int i = 0; i < message_info(type).num_args; i++){
        sprintf(buf,"%f,",va_arg(ap, double));
        msg.append(buf);
    }
    msg.append("TAIL");
    recipient.send(msg);
    if(PRINT_UDP_MESSAGES){
        debugprintf("Sent Message: '%s'\n",msg.c_str());
    }
    va_end(ap);
    //if(message_info(type).requires_confirmation)
    //    confirm_received(recipient, msg);
    return true;
}


bool Messaging::receive_message(UDPSocket& sender, MessageType type, ...){
    va_list ap;
    va_start(ap, type);
    std::string buf;
    //if(!sender.recv(buf)){
    if(!sender.recv(buf,MSG_PEEK)){
        //printf("NO RECEPTION\n");
        return false;
    }
    int front, back, i;
    // get data segment
    front = buf.find("HEAD") + 4;
    back = buf.find("TAIL", front);
    if( !((front >= 0) && (back > 0))){
        //printf("NO FRONT NO BACK\n");
        return false;
    }
    std::string data = buf.substr(front, back-front);
    int flag_loc = data.find(message_info(type).flag);
    if(flag_loc < 0){
        return false;
    }
    int var_start = data.find(":");
    data = data.substr(var_start+1,data.size()-(var_start+1));
    
    for(i=0;i<message_info(type).num_args;i++) {
        double* var;
        if(sizeof(void*)==sizeof(int))
            var = (double*) va_arg(ap, int);
        else
            var = (double*) va_arg(ap, long);
        int comma_loc = data.find(",");
        if(comma_loc < 0){
            //printf("Couldn't find %dth comma\n",i);
            return false;
            }
        *var = strtod(data.substr(0, comma_loc).c_str(),NULL);
        data = data.substr(comma_loc+1, data.size()-(comma_loc+1));
    }
    if(PRINT_UDP_MESSAGES){
        debugprintf("Received Message: '%s'\n",buf.c_str());
    }
    va_end(ap);
    //if(message_info(type).requires_confirmation)
    //    send_message(sender, RECEIVED);
    sender.recv(buf); //Actually pull the buffer
    return true;
   
}

void Messaging::confirm_received(UDPSocket& recipient, std::string msg){
    timeval last_sent, now;
    gettimeofday(&last_sent, NULL);
    gettimeofday(&now, NULL);
    if(PRINT_UDP_MESSAGES){
        debugprintf("Awaiting confirmation of message receival");
    }
    while(!receive_message(recipient, RECEIVED)){
        /*gettimeofday(&now, NULL);
        if(timediff(now,last_sent) > RESEND_TIME){
            //recipient.send(msg);
            if(PRINT_UDP_MESSAGES)
                debugprintf(" . ");
            gettimeofday(&last_sent, NULL);
        }
        */
        usleep(1000); //Added so we don't overload the system.
    }
    if(PRINT_UDP_MESSAGES){
        debugprintf("confirmation received!\n");
    }
}


Messaging::MessageInfo Messaging::message_info(MessageType type){
    switch(type){
        case HELLO:
            return (MessageInfo){0, "hello", false};
        case RECEIVED:
            return (MessageInfo){0, "received", false};
        case WAIT:
            return (MessageInfo){0, "wait", false};
        case RESUME:
            return (MessageInfo){0, "resume", false};
        case THREAD_REQUEST:
            return (MessageInfo){0, "get_thread", true}; //FIXME GIVE SLAVE NUM
        case POSITION_REQUEST:
            return (MessageInfo){0, "get_tip", true}; //FIXME GIVE SLAVE NUM
        case CALIBRATION_REQUEST:
            return (MessageInfo){6, "get_calib", true}; //slave1[x,y,z], slave2[x,y,z]
        case THREAD_RESPONSE:
            return (MessageInfo){3, "return_thread", true};
        case POSITION_RESPONSE:
            return (MessageInfo){3, "return_tip", true};
        case CALIBRATION_RESPONSE:
            return (MessageInfo){0, "return_calib", true};
        case NEXT_WAYPOINT_REQUEST:
            return (MessageInfo){0, "move_your_ass", true};
        case WAYPOINT_RESPONSE:
            return (MessageInfo){7, "waypoint_pos", true};
        case START_TRAJECTORY_REQUEST:
            return (MessageInfo){0, "start_new_trajectory", true};
        case END_TRAJECTORY_REQUEST:
            return (MessageInfo){0, "end_new_trajectory", true};
        case DATA_TRAJECTORY_REQUEST:
            return (MessageInfo){8, "trajectory_data", true};
        case TRAJECTORY_RESPONSE:
            return (MessageInfo){0, "traj_exec_complete", true};
    }
    return (MessageInfo){0, "null", false};        
}


#ifndef _MESSAGING_H_
#define _MESSAGING_H_

#include "../UDPSocket.h"
//#include "shared.h"
#include "debugging.h"
#include <string>
#include <cstring>
#include <vector>
#include <iostream>
#include <sys/time.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
//#include "cwmtx.h"
//#include "util.h"
#define VISION_IP   "192.168.1.110 UDP"
#define CONTROLLER_IP "192.168.1.108 UDP"
#define VISION_IN_PORT_SLAVE1   9004
#define VISION_IN_PORT_SLAVE2   9005
#define CONTROLLER_IN_PORT_SLAVE1  9006
#define CONTROLLER_IN_PORT_SLAVE2 9007
#define VISION_IN_PORT 9004         //kept these in because of calibration
#define CONTROLLER_IN_PORT 9005
#define BUDDY_IP "192.168.1.110 UDP"
#define BUDDY_INCOMING_PORT 9292
#define BUDDY_OUTGOING_PORT 9293


class Messaging{

    public:
        
    enum MessageType{
        /* Generic Handshake Messages */
        HELLO, RECEIVED, WAIT, RESUME,
        /* Specific Slave->Vision Messages */
        THREAD_REQUEST, POSITION_REQUEST, CALIBRATION_REQUEST,
        /* Specific Vision->Slave Messages */
        THREAD_RESPONSE, POSITION_RESPONSE, CALIBRATION_RESPONSE,
        /* Specific Buddy->Slave Messages */
        NEXT_WAYPOINT_REQUEST,START_TRAJECTORY_REQUEST,END_TRAJECTORY_REQUEST,DATA_TRAJECTORY_REQUEST,
        /* Specific Slave->Buddy Messages */
        WAYPOINT_RESPONSE, TRAJECTORY_RESPONSE,
    };
    
    typedef struct MessageInfo{
        int num_args;
        const char* flag;
        bool requires_confirmation;
    } MessageInfo;
    
    /* Send a message with variable number of arguments. true if succeeded*/
    static bool send_message(UDPSocket& recipient, MessageType type, ...);
    /* Receive a message, with ... representing the address of variables to populate */
    static bool receive_message(UDPSocket& sender, MessageType type, ...);

    static MessageInfo message_info(MessageType type);
    
    static void confirm_received(UDPSocket& recipient, std::string msg);

};

#endif

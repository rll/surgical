#ifndef _VISUAL_FEEDBACK_H_
#define _VISUAL_FEEDBACK_H_

#include "Socket.h"
#include "shared.h"
#include <string>
#include <cstring>
#include <vector>
#include <iostream>
#include <sys/time.h>
#include <stdlib.h>
#include "cwmtx.h"
#include "messaging.h"

class Visual_feedback{

    public:
    bool        _accepted;
    UDPSocket   _incoming, _outgoing;
    std::vector<std::string>    _last_raw;
    timeval                     _last_response, _last_input;
    
    Visual_feedback(int in_port, int out_port, const char* ip_addr);
    ~Visual_feedback();
    
    bool received();
    bool request_goal_position();
    bool request_current_position();
    bool get_goal_position(double* pos);
    bool get_current_position(double* pos);
};

#endif

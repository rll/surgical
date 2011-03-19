//#include "Socket.h"
#include "UDPSocket.h"
#include "shared.h"
#include <string>
#include <cstring>
#include <vector>
#include <iostream>
#include <sys/time.h>
#include <stdlib.h>
#include "cwmtx.h"

class Master {
public: 

    /** Whether or not a connection is currently active with the masters */
    bool                        _accepted;
    int                         _master_num, _timeout;
    UDPSocket                   _incoming, _outgoing;
    std::vector<std::string>    _last_raw;
    CwMtx::CWTVector<>          _initial_offset, _pose_offset;
    CwMtx::CWTMatrix<>          _rotate, _scale, _scale_rotate, _inverse;
    timeval                     _last_response, _last_input;


    Master(int master_num, int in_port, int out_port, const char* ip_addr);

    ~Master();

    /** Accept a connection from the port used in the constructor */
    bool                        accept();

    /** Create offsets, such that the master's current location is equal
     * to the given slave position */
    bool                        create_offsets(double pitch, double roll, double gross, double x, double y, double z, double grip, double scale);

    /** Return the goal pitch,roll,gross,x,y,z,grip in the slave's 
     * frame specified by the last complete packet sent by the masters.
     * updates _accepted. */
    std::vector<double>         get_destination();

    /** Given the requested x,y,z and the nearest legal x,y,z in
     * the slave's frame, transfers the forces to apply in the masters'
     * frame back to it */
    void                        respond(double* requested, double* safe);

    std::vector<std::string>    parse(std::string buf);

    std::vector<double>         master_to_slave(std::vector<std::string>);
};

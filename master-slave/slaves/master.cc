#include "master.h"

Master::Master(int master_num, int in_port, int out_port, const char* ip_addr){
    _master_num     = master_num;
    _incoming       = UDPSocket();
    _outgoing       = UDPSocket();
    _accepted       = false;
    _timeout        = 5;
    _initial_offset = CwMtx::CWTVector<>(7);
    _initial_offset.fill(0.0);
    _pose_offset    = CwMtx::CWTVector<>(7);
    _pose_offset.fill(0.0);
    for(int i=0;i<7;i++){
        _last_raw.push_back("1.0");
    }
    _scale          = CwMtx::CWTSquareMatrix<>(7);
    _scale.fill(0.0);
    _inverse        = CwMtx::CWTSquareMatrix<>(7);
    _inverse.fill(0.0);
    _scale_rotate   = CwMtx::CWTSquareMatrix<>(7);
    _scale_rotate.fill(0.0);
    _rotate         = CwMtx::CWTSquareMatrix<>(7);
    _rotate.fill(0.0);
    gettimeofday(&_last_response, NULL);

    // from x,y,z,roll,pitch,gross,button to
    // pitch, roll, gross, x, y, z, button (including
    // direction switching)

    // pitch, roll, gross rotation
    _rotate[0][3] =   0;_rotate[0][4] =  -1;_rotate[0][5] =   0;
    _rotate[1][3] =   0;_rotate[1][4] =   0;_rotate[1][5] =  -1;
    _rotate[2][3] =  -1;_rotate[2][4] =   0;_rotate[2][5] =   0;

    // x,y,z rotation
    _rotate[3][0] =-1;_rotate[3][1] = 0;_rotate[3][2] =  0;
    _rotate[4][0] = 0;_rotate[4][1] = 1;_rotate[4][2] =  0;
    _rotate[5][0] = 0;_rotate[5][1] = 0;_rotate[5][2] = -1;
    
    // grip
    _rotate[6][6] = 1;

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

Master::~Master() {

}

bool Master::accept() {
    std::string buf;
    _accepted = _incoming.recv(buf);
    return _accepted;
}

std::vector<double> Master::get_destination() {
    std::string buf;
    std::vector<double> result = std::vector<double>();
    if (_accepted) {
        int status = _incoming.recv(buf);
        std::vector<std::string> parsed = parse(buf);
        result = master_to_slave(parsed);
            
        // Break connection after 3 sec of inactivity
        if (status > 0) {
            gettimeofday(&_last_input, NULL);
        } else if (status == 0) {
            timeval now;
            gettimeofday(&now, NULL);
            double diff = ( ( (double) now.tv_sec)*1000000+now.tv_usec ) - ( ( (double) _last_input.tv_sec)*1000000+_last_input.tv_usec );
            if(diff > 3000000)
                _accepted = 0;
        } else {
            _accepted = 0;
        }
    }
    return result;
}

bool Master::create_offsets(double pitch, double roll, double gross, double x, double y, double z, double grip, double scale) {
    short i;
    _pose_offset.fill(0.0); _initial_offset.fill(0.0);
    _initial_offset[0] = pitch;
    _initial_offset[1] = roll;
    _initial_offset[2] = gross;
    _initial_offset[3] = x;
    _initial_offset[4] = y;
    _initial_offset[5] = z;
    //_initial_offset[6] = grip;
    _initial_offset[6] = 0;
    
    std::string buf;
    bool success = 0;
    int timeout = _timeout;
    while (_accepted && !success) {
        int status = _incoming.recv(buf);
        if (status > 0) {
            if (timeout > 1) {
                _last_raw = parse(buf);
                timeout -= 1;
            } else {
                // First packet is always all 0, so delay
                std::vector<std::string> vals = parse(buf);
                _last_raw = parse(buf);

                for(i=0;i<6;i++) //DON'T WANT GRIP TO HAVE OFFSET
                    _pose_offset[i] = strtod(vals.at(i+1).c_str(), NULL);
                
                scale = fabs(scale);

                // Create scaling matrix
                //_scale[0][0] = 1.1
                _scale[0][0] =      1.2;_scale[1][1] =     1.1;_scale[2][2] =       2;
                _scale[3][3] =    scale;_scale[4][4] =   scale;_scale[5][5] =   scale;
                _scale[6][6] =        GRIP_ON;

                // Scale Rotate
                _scale_rotate = _scale * _rotate;
                
                // Created response matrix
                CwMtx::CWTSquareMatrix<> rotate = _scale*_rotate;
                rotate.makeInverse();
                _inverse = rotate;

                // done
                gettimeofday(&_last_input, NULL);
                success = 1;
            }
        } else if (status == 0){
            continue;
        } else {
            _accepted = 0;
        }
    }
    return success;
}

std::vector<std::string> Master::parse(std::string buf) {
    int front, back, i;
    // get data segment
    front = buf.find("HEAD") + 4;
    back = buf.find("TAIL", front);
    //std::cout << buf << std::endl;
    if( !((front >= 0) && (back > 0)))
        return _last_raw;
    std::string data = buf.substr(front, back-front);

    // parse the std::string
    std::vector<std::string> result;
    for(i=0;i<8;i++) {
        int comma_loc = data.find(",");
        result.push_back(data.substr(0, comma_loc));
        data = data.substr(comma_loc+1, data.size()-(comma_loc+1));
    }
    _last_raw = result;
    return result;
}

std::vector<double> Master::master_to_slave(std::vector<std::string> raw) {
    short i;
    CwMtx::CWTVector<> pos(7);
    for(i=0;i<7;i++)
        pos[i] = strtod(raw.at(i+1).c_str(), NULL);
    //ADDED BY STEPHEN
    //pos[6] += fabs((pos[5] - _pose_offset[5])*0.04);
    pos = pos - _pose_offset;
    pos = _scale_rotate * pos;
    pos = pos + _initial_offset;

    std::vector<double> result;
    for(i=0;i<7;i++)
        result.push_back(pos[i]);
    return result;
}

void Master::respond(double* requested, double* safe) {
    timeval now;
    gettimeofday(&now, NULL);
    double diff = ( ( (double) now.tv_sec)*1000000+now.tv_usec ) - ( ( (double) _last_response.tv_sec)*1000000+_last_response.tv_usec );
    if (diff > 10000){
        CwMtx::CWTVector<double> orig(7);
        CwMtx::CWTVector<double> actual(7);
        orig.fill(0); actual.fill(0);
        
        for(int i=3;i<6;i++) {
            orig[i]     = requested[i];
            actual[i]   = safe[i];
        }

        CwMtx::CWTMatrix<> diff = actual-orig;
        diff = _inverse * diff;

        char msg[512];
        memset(msg, 0, sizeof(char));
        sprintf(msg, "HEAD%d,%f,%f,%f,TAIL", 1, diff[0][0], diff[1][0], diff[2][0]);
        //std::cout << msg << std::endl;
        _outgoing.send(msg);
        gettimeofday(&_last_response, NULL);
    }
}

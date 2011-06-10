/* thread_socket_interface.h */

#include <stdlib.h>
#include <iostream>
#include <cstring>
#include <string>
#include "../utils/sockets/UDPSocket.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <Eigen/Core>
//#include <HD/hd.h>
//#include <HDU/hdu.h>
//#include <HDU/hduError.h>
//#include <HDU/hduVector.h>
//#include <HDU/hduMatrix.h>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

#define RECEIVE_PORT 9000
#define SEND_PORT 9001

using namespace std;

void parse(string buf, vector<string> &vect);

void connectionInit();

void getDeviceState (double start_proxyxform[], bool &start_proxybutton, double end_proxyxform[], bool &end_proxybutton);

void getDeviceState (Vector3d &leftPosition, Matrix3d &leftRotation, Vector3d &rightPosition, Matrix3d &rightRotation);

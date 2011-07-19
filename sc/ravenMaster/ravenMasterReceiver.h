#include "utils/PracticalSocket/PracticalSocket.h"
#include "utils/Log/Log.h"
#include "utils/_kbhit.h"
#include "utils/_getch.h"
#include "UDPAddressPort.h"
#include "structs.h"
#include "DS0.h"

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string>
//#include <vector>
//#include <Eigen/Core>
//#include <Eigen/Geometry>

// import most common Eigen types
//USING_PART_OF_NAMESPACE_EIGEN

using namespace std;

#define THOUSAND 1000
#define MILLION  (THOUSAND*THOUSAND)
#define round(x) (x)

void socketInit();
bool recv_UDP_from_slave();
bool recv_UDP_from_send();
void send_UDP_to_send();
unsigned int byteswap (unsigned int nLongNumber);



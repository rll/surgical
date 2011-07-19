#include "utils/PracticalSocket/PracticalSocket.h"
#include "utils/Log/Log.h"
#include "sequence.h"
#include "UDPAddressPort.h"
#include "structs.h"
#include "DS0.h"
#include "itp_teleoperation.h"

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

using namespace std;

#define THOUSAND 1000
#define MILLION  (THOUSAND*THOUSAND)
#define round(x) (x)

void socketInit();
void msgHeaderInit();
void update_UDP_to_slave(double pos[2][3], double rot[2][3], int bttn[2]);
bool recv_UDP_from_cmd();
void send_UDP_to_slave();
bool recv_UDP_from_recv();
void send_UDP_to_recv();
bool msgHeaderZero();
int checksumUDPData();

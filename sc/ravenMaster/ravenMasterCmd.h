#include "utils/PracticalSocket/PracticalSocket.h"
#include "utils/Log/Log.h"
#include "UDPAddressPort.h"
#include "structs.h"

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>

using namespace std;
using boost::lexical_cast;
using boost::bad_lexical_cast;

void socketInit();
void send_UDP_to_sender();
double cast_str(string s);
int cast_str_int(string s);
void printHelp();
void printState();

#include <stdio.h> 
#include <cstring> 
#include "DS0.h"
#include <iostream>
#include <fstream>
#include <string> 
#include <vector> 

using namespace std;

#define RAD2DEG *180.0/M_PI
#define MICRORADS_PER_RAD 1000000
#define MICROMETERS_PER_METER 1000000 

unsigned int byteswap (unsigned int nLongNumber) {
	return (((nLongNumber&0x000000FF)<<24)+((nLongNumber&0x0000FF00)<<8)+
					((nLongNumber&0x00FF0000)>>8)+((nLongNumber&0xFF000000)>>24));
}

void parseLine(const char* line, struct robot_device &rdev) {
  memset(&rdev, 0, sizeof(rdev));
  memcpy(&rdev, line, sizeof(rdev)); 

  if (rdev.Ox12345678 == 0x12345678) {
    //printf("fine\n");
  } else if (rdev.Ox12345678 == 0x78563412) {
    //printf("opposite endianness\n");
    unsigned int *ip_first, *ip_last, *ip;
    if ( ((char) (rdev.Ox12345678)) != 0x78) // Data received in other endianness
    {				
      ip_first = (unsigned int*) &(rdev.Ox12345678);
      ip_last  = (unsigned int*) &(rdev.ending_int);
      for (ip = ip_first; ip <= ip_last; ip++)
      {
        *ip = byteswap(*ip);
      }
    }
  } else {
    //printf("ERROR: Unsupported endianness\n");
  }

    int i = 2;
    printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", 
        ((double)rdev.mech[i].pos.x) / MICROMETERS_PER_METER,
        ((double)rdev.mech[i].pos.y) / MICROMETERS_PER_METER,
        ((double)rdev.mech[i].pos.z) / MICROMETERS_PER_METER,
        ((double)rdev.mech[i].ori.yaw) / MICRORADS_PER_RAD,
        ((double)rdev.mech[i].ori.pitch) / MICRORADS_PER_RAD,
        ((double)rdev.mech[i].ori.roll) / MICRORADS_PER_RAD,
        ((double)rdev.mech[i].ori.R[0][0]),         
        ((double)rdev.mech[i].ori.R[0][1]),        
        ((double)rdev.mech[i].ori.R[0][2]),        
        ((double)rdev.mech[i].ori.R[1][0]),         
        ((double)rdev.mech[i].ori.R[1][1]),        
        ((double)rdev.mech[i].ori.R[1][2]),         
        ((double)rdev.mech[i].ori.R[2][0]),         
        ((double)rdev.mech[i].ori.R[2][1]),        
        ((double)rdev.mech[i].ori.R[2][2])         
        );

}

void loadParsedFile(const char* fileName, vector<vector<double> >& data) {
  ifstream parsedFile(fileName, ios::in);
  data.resize(0); 
  if (parsedFile.good()) { 
    cout << "Log file open success" << endl; 
    while(!parsedFile.eof()) {
      vector<double> point; 
      for (int i = 0; i < 15; i++) { 
        double value;
        parsedFile >> value; 
        point.push_back(value); 
      }
      data.push_back(point); 
    }
    // last point is bogus so pop it
    data.pop_back(); 
  }
}

int main(int argc, char* argv[]) {
	/* initialize glut */
  char* logFileName = argv[1];
  ifstream logFile(logFileName, ios::in | ios::binary); 
  
  if(logFile.good()) {
    struct robot_device rdev; 
    char buffer[sizeof(struct robot_device)]; 
    while(!logFile.eof()) {
      logFile.read(buffer, sizeof(struct robot_device));
      parseLine(buffer, rdev); 
    }
  }

  logFile.close(); 

  return 0; 
}

// shared.h
// constants common to both robots; needed for kinematics and control

#ifndef __SHARED_H__
#define __SHARED_H__

#include <boost/shared_ptr.hpp>
#include <Galil.h>
#include <stdio.h>
#include <sys/io.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <sys/time.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <limits>
#include <map>
#include "cwmtx.h"
#include "debugging.h"
#include "pid_settings.h"

#define num_actuators 8
#define num_sensors 8
#define num_slaves 2
#define standard_update_interval 1000

//Files for LQR
#define DATA_FILEPATH "lqr_input"
enum{PID,LQR,PID_AND_LQR,VISUAL,VISUAL_SERVO_ONLY,VISUAL_MULTI_PLANNER_KNOTTIE};

// Degrees of Freedom mapping.  All position controls use double* of this format.
#define num_dof 7
#define iPITCH          0
#define iEulerZ         0
#define iROLL           1
#define iTILT           iROLL
#define iEulerY         1
#define iGROSS          2
#define iEulerX         2
#define iX              3
#define iSTICK_GROSS    3
#define iY              4
#define iSTICK_PITCH    4
#define iZ              5
#define iSTICK_LENGTH   5
#define iGRIP           6

//Motor index mapping
#define mPITCH 0
#define mGROSS 3
#define mYAW1 1
#define mYAW2 2
#define mBASE_LEFT 4
#define mBASE_REAR 5
#define mBASE_RIGHT 6


// encoder counts per rad for wrist motors (old setup)
#define R_COUNT 8725.0

// counts per mm for base motors
#define M_COUNT 80.0 * 1.2

//New definitions, in terms of high resolution and low resolution motors
#define R_COUNT_HIGH 2112/(2*M_PI)
#define R_COUNT_LOW 528/(2*M_PI)

//New total
#define R_COUNT_WRIST 13248/(2*M_PI)

// workspace limits (old setup)
#define PITCH_MIN           0
#define PITCH_MAX           M_PI/2
#define ROLL_MIN            -3*M_PI/2
#define ROLL_MAX            0
#define GROSS_MIN           -23*M_PI/6
#define GROSS_MAX           0
#define ANGLE_MAX           1.11701072  //64 degrees
#define ARM_MIN             90
#define CONE_SPHERE_BOUND   184.252779
#define ARM_MAX             205
#define RADIUS_HEIGHT_RATIO 89.8660853/184.252779

// Default tension for wrist
#define DEFAULT_TENSION 0.0

// Voltage Limits
#define MAX_VOLTAGE_BASE 9
#define TORQUE_LIMIT_WRIST 0.60
#define TORQUE_LIMIT_GROSS 0.75
#define PEAK_TORQUE_WRIST 0.65
#define PEAK_TORQUE_GROSS 1.00
#define MAX_VOLTAGE_WRIST TORQUE_LIMIT_WRIST


// define master space
#define MAX_XY              -1.0
#define MAX_Z               -2.7

// For base kinematics
#define CROSS_LENGTH 431.8
#define STICK_LENGTH 363.2
#define BOOM_LENGTH 548.4
#define BOOM_ALTITUDE 520.7
#define LINK_LENGTH_WITH_PINION 182.9
#define LINK_ALTITUDE 179.1
#define PINION_DROOP_ANGLE 0.099

// Connection stuff for Accelera
#define SLAVE1_IP   "192.168.1.106 UDP"
#define SLAVE2_IP   "192.168.1.107 UDP"
#define MY_IP       "192.168.1.108 UDP"
#define MASTERS_IP  "192.168.1.109 UDP"

// PID 
enum{iKP,iKI,iKD};




// For old setup with ISA boards
#define MIRANOVA_BASE 0x230
#define QUATECH_BASE 0x300
#define QUATECH_DABASE1 (QUATECH_BASE+0x00) /* address of D/A converter module */
#define QUATECH_DABASE2 (QUATECH_BASE+0x04) /* address of D/A converter module */
#define QUATECH_ZEROVOLT 0x80
#define QUATECH_DIGBASE  (QUATECH_BASE+0x08) /* address of digital output */
#endif

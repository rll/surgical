/* Helpful tools for debugging */
#ifndef __DEBUGGING_H__
#define __DEBUGGING_H__

#define PRINT_COMMANDS false
#define PRINT_COMMAND_FAILURES false
#define NO_VOLTAGES true
#define PRINT_MAX_VOLTAGE false
#define PRINT_SETPOINT_VS_NEAREST_LEGAL false
#define MAKE_ALL_POINTS_LEGAL false
#define PRINT_ENCODER_OFFSET_CHANGES false
#define PRINT_WHEN_USING_ELBOW_COOR false
#define PRINT_PID_VALUES false
#define PRINT_GRIP_POS_VS_GOAL false
#define PRINT_UDP_MESSAGES true
#define USE_LOW_ACCELERATION true
#define USE_GALIL_PID true

#define LOGGING_PREFIX "logs/"


void debugprintf(const char* format, ...);




#endif

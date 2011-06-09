#include "clock.h"


#ifdef WIN32

#include <windows.h> 
// to convert to seconds
#define CLOCK_SCALE 0.0001024

#else

#include <sys/time.h>
#include <time.h>

#endif



static long unsigned int startTime;

/*
 * Starts the clock!  Call this once at the beginning of the program.
 * Calling again will reset the clock to 0;  but doing so is not 
 * thread-safe if other threads may be calling GetClock(); (which
 * is thread-safe since it only reads values and calls thread-safe
 * functions in the kernel).
 */
// time in units of seconds since some time in the past; bitshift by 10 --> in units of 2^10 * 100 ns
void StartClock() {
	//determine start time

#ifdef WIN32
  FILETIME fileTime;
  GetSystemTimeAsFileTime(&fileTime);
  startTime = (fileTime.dwHighDateTime << 22 | fileTime.dwLowDateTime >> 10) & 0xFFFFFFFF;
#else
  struct timeval startTimeStruct;
  gettimeofday(&startTimeStruct, NULL);
  startTime = startTimeStruct.tv_sec*(long unsigned int)(1e6) + startTimeStruct.tv_usec; 
#endif
}

/*
 * Returns the current time since the call to StartClock();
 */
double GetClock() {
#ifdef WIN32
	FILETIME fileTime; unsigned long int curTime;
	GetSystemTimeAsFileTime(&fileTime);
	curTime = (fileTime.dwHighDateTime << 22 | fileTime.dwLowDateTime >> 10) & 0xFFFFFFFF;
	return (1e-6 * (10*(double)(curTime - startTime)*CLOCK_SCALE));
#else
	struct timeval startTimeStruct; unsigned long int curTime;
	gettimeofday(&startTimeStruct, NULL);
	curTime = startTimeStruct.tv_sec*(long unsigned int)(1e6) + startTimeStruct.tv_usec; 
	return (double) (1e-6)*(curTime - startTime);
#endif
}


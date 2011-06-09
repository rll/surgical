#include "clock2.h"
#include <iostream>

#ifdef WIN32
	#include <windows.h> 
// to convert to seconds
// #define CLOCK_SCALE 0.0001024
	double sys_timer_freq;
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
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	if (freq.QuadPart == 0)
	{
		std::cout << "ARGH! This system does not (for whatever reason) have a performance timer. My program won't work." << std::endl;
		exit(-1);
	}
	//	TRACE("System timer is running at %.1f MHz\n", (float)freq.LowPart / 1000000.0);
	sys_timer_freq = (double)freq.QuadPart;
	LARGE_INTEGER curtime;
	QueryPerformanceCounter(&curtime);
	startTime = (double)curtime.QuadPart/sys_timer_freq;
	std::cout << "sys_timer_freq=" << sys_timer_freq << ", startTime=" << startTime << std::endl;
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
	LARGE_INTEGER counter;
	QueryPerformanceCounter(&counter);
	return ((double)counter.QuadPart / sys_timer_freq) - startTime;
#else
	struct timeval startTimeStruct; unsigned long int curTime;
	gettimeofday(&startTimeStruct, NULL);
	curTime = startTimeStruct.tv_sec*(long unsigned int)(1e6) + startTimeStruct.tv_usec; 
	return (double) (1e-6)*(curTime - startTime);
#endif
}


#ifndef WOODWARD_H
#define WOODWARD_H

// THREADS / MUTEX
typedef struct WdThreadCDT *WdThreadADT;
typedef struct WdMutexCDT *WdMutexADT;

WdThreadADT wdCreateThread(void*(*func)(void *), void *ptr);
void wdJoinThread(WdThreadADT thread);
//void wdDeleteThread(WdThreadADT);

WdMutexADT wdCreateMutex();
void wdAcquireMutex(WdMutexADT mutex);
void wdReleaseMutex(WdMutexADT mutex);
//void wdDeleteMutex

void wdSleep(unsigned int ms);
//void wduSleep(unsigned int useconds);

// MACROS
// min
#ifndef min
#define min(a,b) (((a)<(b))?(a):(b))
#endif // min

// max
#ifndef max
#define max(a,b) (((a)>(b))?(a):(b))
#endif // max

// PI
#ifndef PI
#define PI 3.14159265
#endif // PI



#endif // WOODWARD_H

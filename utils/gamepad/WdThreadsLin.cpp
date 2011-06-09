#include "Woodward.h"
#include <pthread.h>        // Threads
#include <unistd.h>

struct WdThreadCDT {
  pthread_t hndl;
};

struct WdMutexCDT {
  pthread_mutex_t hndl;
};

WdThreadADT wdCreateThread(void*(*func)(void *), void *ptr) {
  WdThreadADT thread = new WdThreadCDT;
  pthread_create(&thread->hndl, NULL, func, ptr);
  return thread;
}

void wdJoinThread(WdThreadADT thread) {
  pthread_join(thread->hndl, NULL);
}

WdMutexADT wdCreateMutex() {
  WdMutexADT mutex = new WdMutexCDT;
  pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
  mutex->hndl = mu;
  return mutex; 
}

void wdAcquireMutex(WdMutexADT mutex) {
  pthread_mutex_lock(&mutex->hndl);
};

void wdReleaseMutex(WdMutexADT mutex) {
  pthread_mutex_unlock(&mutex->hndl);
};

// MISC
void wdSleep(unsigned int ms) {
  usleep(ms*1000);
}

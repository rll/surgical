#include <fstream>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/time.h>

using namespace std;

class Log {
  public:
  	Log();
    Log(char* filename, bool concatTimeStamp=true, bool bin=false);
    ~Log();
    void Write(const char* logline, ...);
    void WriteBinary(const char* data, unsigned int length);
  private:
    ofstream m_stream;
    bool binary;
    char time_s[25];
};

void timestamp(char* time_s);


class Timer {
  public: 
    Timer() {
      gettimeofday(&start_tv, NULL); 
    }
    void restart() {
      gettimeofday(&start_tv, NULL);
    }
    double elapsed() {
      gettimeofday(&tv, NULL); 
      return  (tv.tv_sec - start_tv.tv_sec) +
        (tv.tv_usec - start_tv.tv_usec) / 1000000.0;
    }
    
    double currentTime() {
    	gettimeofday(&tv, NULL); 
    	return (tv.tv_sec + tv.tv_usec / 1000000.0);     
    }

  private:
    struct timeval tv;
    struct timeval start_tv;

};


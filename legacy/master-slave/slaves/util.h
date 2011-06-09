#ifndef __UTIL_H__
#define __UTIL_H__

#include "shared.h"
#include <termios.h>
#include <asm/ioctls.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stropts.h>

// Mathematic utilities
double sq(double x);
double angle_law_of_cos(double a, double b, double c);
double side_law_of_cos(double a, double b, double angle);

int _kbhit();
int _getch(void);
void getperm(int addr, int num);

/** Builds commands for accelera
 * base     = ie: PA, MG,...
 * channel  = A/B/C/D/E/F/G/H
 * arg      = number argument, if it there is one.  ignored in type 'l'
 * type     = 'c': comma seperated (uses arg); 'l': letters (channel is arg) 
 * argtype  = 'i': int, 'd': floating */
std::string build_cmd(const char* base, const char channel, const char type = 'l', double arg = 0 , const char argtype = 'i');

void miranova_init_channel(int channel, long ncount, int quadmode);
long miranova_read(int channel, int &status);
void quatech_outdac8_s(int channel, int value, int base);
void quatech_vout(int channel, double volt, int slave_num);

void copy_pos(const double* original, double* copy);
void add_pos(const double* toAdd1, const double* toAdd2, double* sum);

/** return difference in time in microseconds */
double timediff(timeval now, timeval then);

class Timer {
    public:

    Timer();
    ~Timer();
    
    timeval last_call;

    double dt();
};

class Time_file : public std::ifstream {
    public:
    timeval         last_call;
    double          remainder,last_load;
    std::vector<double>  last_data;
    int             data_size;

    Time_file(const char* filename, int d_size);
    ~Time_file();

    std::vector<double> data();
    void reset_time();

    private:
    void get_data();
    bool update_now();
};


#endif

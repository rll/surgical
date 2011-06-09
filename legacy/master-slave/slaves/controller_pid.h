#ifndef _CONTROLLER_PID_H_
#define _CONTROLLER_PID_H_

#include "controller.h"
#include "LowPassFilter.h"
#include "util.h"

class Controller_pid : public Controller {
public:
    // Low Pass filter params
    LowPassFilter**     _f;
    double*             _kp;
    double*             _ki;
    double*             _kd;
    std::list<double>*  _errors;
    timeval*            _now;
    timeval*            _last;
    timeval            _last_jolt;
    LowPassFilter*      _overall;
    double*             _last_error;
    
    
    Controller_pid(int slave_num, int statesize, int outputsize, double* kp, double* ki, double* kd);

    ~Controller_pid();

    int control(const double* state, double* goal, double* out);
    
    void set_constants(double* kp, double* ki, double* kd);

};

#endif

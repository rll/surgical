#include "controller_pid.h"


//[b,a] = cheby2(5,20,0.04)
//double b[6] = { 0.026196188550650,  -0.076945102609320,   0.050769599839752,   0.050769599839753,  -0.076945102609320,   0.026196188550650 };
//double a[6] = { 1.000000000000000,  -4.593368688459326,   8.454163420301082,  -7.792039423922514,   3.595842453885893,  -0.664556390242970 };
//[b,a] = cheb2(8,20,0.04)
//double b[9] = { 0.076833176499288,  -0.578813428945648,   1.938998271763937,  -3.775933365603108,   4.677831239899627,  -3.775933365603104,  1.938998271763931,  -0.578813428945646,   0.076833176499288};
//double a[9] = { 1.000000000000000,  -7.229343074530456,  22.901989529901613, -41.522450568838480,  47.122555875010086, -34.276811529023519, 15.606136091292942,  -4.066333181561192,   0.464257405077563 };

Controller_pid::Controller_pid(int slave_num, int statesize, int outputsize, double* kp, double* ki, double* kd) : Controller(slave_num, statesize, outputsize) {
    assert (state_size == output_size);     // each motor is independent

    //[b,a] = cheby2(5,20,.2)
    double b[6] = { 0.084226357716145,  -0.138612396592324,   0.087352881630195,   0.087352881630195,  -0.138612396592324,   0.084226357716146 };
    double a[6] = { 1.000000000000000,  -2.966123533950475,   3.815765378671260,  -2.543840645716643,   0.878222628350575,  -0.118090141846685 };

    double wrist_b[6] = {0.052629628146691,  -0.137957204983314,   0.086865831894144,   0.086865831894143,  -0.137957204983313,   0.052629628146691};
    double wrist_a[6] = {1.000000000000000,  -3.985059962505463,   6.438334626234461,  -5.253966948692174,   2.160820180917132,  -0.357051385838915};

    _f      = new LowPassFilter*[state_size];
    _kp     = new double[state_size];
    _ki     = new double[state_size];
    _kd     = new double[state_size];
    _errors = new std::list<double>[state_size];
    _now    = new timeval[state_size];
    _last   = new timeval[state_size];
    _overall= new LowPassFilter(a,b,6,state_size);
    
    _last_error = new double[state_size];
    for(int i = 0; i < state_size; i++){
        _last_error[i] = 0.0;
    }

    for(int i=0;i<state_size;i++) {
        _kp[i] = kp[i];
        _ki[i] = ki[i];
        _kd[i] = kd[i];

        _errors[i].assign(1000, 0);

        gettimeofday(&_last[i], NULL);
        
        gettimeofday(&_last_jolt, NULL);
        _f[i] = new LowPassFilter( a, b, 6, 1);
    }
}
    
Controller_pid::~Controller_pid() {
    delete[] _kp;
    delete[] _ki;
    delete[] _kd;
    delete[] _errors;
    delete[] _now;
    delete[] _last;

    for(int i=0;i<state_size;i++) {
        delete _f[i];
    }
    delete[] _f;
}

int Controller_pid::control(const double* state, double* goal, double* out) {
    double raw_out[state_size];
    bool jolted = false;
    for(int i=0;i<state_size;i++) {
        double error, integral, derivative;
        
        // Error
        error = goal[i] - state[i];

        // Derivative of error (low pass filtered)
        double raw_derivative = error - _errors[i].back();

        gettimeofday(&_now[i], NULL);
        double diff = (((double) _now[i].tv_sec) * 1000000 + _now[i].tv_usec) - (((double) _last[i].tv_sec) * 1000000 + _last[i].tv_usec);
        diff /= 1000000;                        // diff in seconds
        raw_derivative = raw_derivative/diff;   // units/sec
        gettimeofday(&_last[i], NULL);

        _errors[i].push_back(error);
        _errors[i].pop_front();
        _f[i]->Update(&raw_derivative, &derivative);
        
        // Integral of error over n steps
        double error_sum = 0;
        std::list<double>::iterator iter;
        for(iter=_errors[i].begin();iter!=_errors[i].end();iter++){
            error_sum += *iter;
        }
        integral = error_sum + error;
        
        /*integral = _last_error[i] + error;
        _last_error[i] += error;
        // output in volts
        if((i == mBASE_LEFT || i == mBASE_REAR || i == mBASE_RIGHT)){
            if(i == mBASE_LEFT)
                debugprintf("Left motor:\n");
            else if(i == mBASE_REAR)
                debugprintf("Rear motor:\n");
            else
                debugprintf("Right motor:\n");
            debugprintf("P: %f * %f = %f\n",_kp[i],error,_kp[i]*error);
            debugprintf("I: %f * %f = %f\n",_ki[i],integral,_ki[i]*integral);
            debugprintf("D: %f * %f = %f\n",_kd[i],derivative,_kd[i]*derivative);
         }
        */
	    raw_out[i] = _kp[i] * error + _kd[i] * derivative + _ki[i] * integral;
	    
	    /* Added for pitch getting stuck */
	    double jolt_amt = 0;
        if(PITCH_JOLT && i == 0 && timediff(_now[i], _last_jolt) >= JOLT_PERIOD){
            if(error > JOLT_MIN_ERROR && fabs(derivative) < JOLT_MAX_DERIVATIVE && fabs(error - error_sum / 1000.0) < JOLT_MAX_VARIANCE){
                if(error > 0)
                    raw_out[i] = JOLT_VOLTAGE;
                else
                    raw_out[i] = -1 * JOLT_VOLTAGE;
                gettimeofday(&_last_jolt, NULL);
                jolted = true;
            }
        }

    }
    //std::cout << "Voltages prescribed: ";
    //for(int i=0;i<state_size;i++) {
    //    std::cout << out[i] << ",";
    //}
    //std::cout << std::endl;

    // Smooth total output
    _overall->Update(raw_out, out);
    //in the case of a jolt, we don't want it smoothed.
    if(jolted)
        out[0] = raw_out[0];
    
    last_output.clear();
    for(int i=0;i<state_size;i++) {
        last_output.push_back(out[i]);
    }


    return state_size;
}



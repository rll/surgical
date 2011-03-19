#ifndef _CONTROLLER_LQR_H_
#define _CONTROLLER_LQR_H_

#define TRUE_STATE_SIZE 17      // position, input of all motors plus 1 for bias
#define TRUE_OUTPUT_SIZE 8      // difference in voltage between two timesteps

#include "shared.h"
#include "util.h"
#include "LowPassFilter.h"
#include "controller.h"

class Controller_lqr : public Controller {
    public:

    /** Contains:
     *  CLOCK (microseconds)
     *  K MATRIX (row by row)
     *  TARGET STATE
     *  CLOCK
     *  K MATRIX
     *  ... */
    Time_file*      matrix_data;
    Time_file*      target_data;

    /** If remainder < 0, load next matrix. */
    timeval             last_control;
    CwMtx::CWTMatrix<>* K;
    CwMtx::CWTMatrix<>* target_state;
    double*             last_state;
    double*             last_dv;

    Controller_lqr(int slave, int statesize, int outputsize, const char* matrixfilepath, const char* targetfilepath);
    ~Controller_lqr();

    int     control(const double* state, double* goal, double* out);
    //void    assert_endofline();
    void    load_next_datum();
    //void    load_next_clock();
    void    load_next_matrix();
    void    load_next_target();
    
    void    reset_time();
    bool    is_complete();
};

#endif

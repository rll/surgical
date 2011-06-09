#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "shared.h"

class Controller {
    public:

    /** Which slave this controller is for (start @ 1) */
    int         slave_num;

    /** Number of elements in argument state */
    int         state_size;

    /** Same for output */
    int         output_size;
    std::vector<double> last_output;

    Controller(int slave, int statesize, int outputsize);
    ~Controller();

    /** Puts recommended output into out, which must be of length
     * output_size.  state and goal are of length state_size, where
     * state is the current pose and goal is the desired pose.
     * Returns number of elements copied. */
    virtual int control(const double* state, double* goal, double* out)=0;
    
    virtual bool ignore_log();
    virtual bool log_message(char* msg);
    
    virtual void set_other_controller(boost::shared_ptr<Controller> controller);
};

#endif

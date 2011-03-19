#ifndef __SLAVE_H__
#define __SLAVE_H__

#include "shared.h"     // Many defines
#include "controller.h"
#include "system.h"

#include "utility.h"    // timestamped filenames

#define LOGGING_PREFIX "logs/"

/** A generic slave class with actuators, sensors, controllers, and kinematics objects.
 * One must be aware from the Kinematics object how many inputs are necessary to specify
 * a motor position.  See the implemented Kinematics object to determine how each
 * input is related to the motor. */
class Slave {
    public:
        /** Slave number.  1 or 2. */
        int                 _id;

        /** The previous time voltages were sent out & the number
        * of microseconds to wait until the next. */
        timeval             _previous_time;
        int                 _interval;      // default = 1000 microsec

        /** The controllers, actuators, and encoders */
        boost::shared_ptr<System>       _sys;
        boost::shared_ptr<Controller>   _contr;

        /* logs to file "logs/slaves<timestamp> */
        std::ofstream       _logfile;

        /* Homing position */
        double              home_points[num_slaves][num_dof];

        
//////////////////////////////////////////Initialization/////////////////////////////////////////
        /** Constructor, which expects num_actuator controllers, 
         * num_sensor sensors, and num_actuator actuators in an 
         * array corresponding to the spec above.  Also, the 
         * model (kinematics).  Initializes current slave 
         * pose as 0 on all channels */
        Slave(int id, boost::shared_ptr<Controller> controller, boost::shared_ptr<System> s);

        ~Slave();

        /** Reset a single axis such that its current position is
        * recognized as the one given here */
        void init_setpoint(int index, double value);

        /** Uses init_setpoint() safely to ensure an axis is defined
        * only if all the requisite keys in pose correspond to enough
        * information to set the points.  For example, if you only define
        * x and y, there's no point in setting the base motors as you
        * need z as well. Also, this explicitly allows you to define
        * only some of the motors in the slave, instead of all. */
        virtual void init_setpoints(std::map<int,double> pose, bool use_base_coor=false) = 0;

        /** Homes the slave into a reproducible position */
        virtual void home() = 0;

//////////////////////////////////////////Movement///////////////////////////////////////////////
        /** Must be run at least as often as update_now() 
         * returns true (1 ms).  Sends out voltages to the
         * slave to move it to the specified position if 
         * update_now().  */
        void move_to(double pos[num_dof], bool use_kdl=false, bool block=false);

        /** Move motor towards point specified*/
        void motors_to(double motor_points[num_actuators], bool block=false);
        
        /** a shortcut using motors_to_point to find out where the slave is
         * right now.  If use_base_coor=false, values are only valid if the
         * tip is in the trocar. */
        double* current_pose(double out[num_dof], bool use_base_coor=false, bool use_kdl=false, bool no_cache=false);
        double* current_position(double out[num_actuators], bool no_cache=false);
        double* current_velocity(double out[num_actuators], bool no_cache=false);
        void    block_while_moving(); 

        virtual double* point_to_motors(double in[num_dof], double out[num_actuators], bool use_base_coor=false, bool use_kdl=false) = 0;
        virtual double* motors_to_point(double out[num_actuators], double in[num_dof], bool use_base_coor=false, bool use_kdl=false) = 0;
        virtual double* nearest_legal_point(double in[num_dof], double out[num_dof]) = 0;

        /** More like update_now?().  If returns true, when
         * called, a move_to will send out voltages when called */
        bool update_now();

        /** Internal.  Resets each time voltages are sent
         * to the slave */
        void reset_timer();
/////////////////////////////////////////Logging/////////////////////////////////////////////////
        void start_logging();

        void stop_logging();
        
        void log(const char* msg);
////////////////////////////////////////Etc//////////////////////////////////////////////////////

        void print();

};
#endif

#ifndef __SLAVE_MARK1_H__
#define __SLAVE_MARK1_H__

#include "slave.h"

#define LOGGING_PREFIX "logs/"

/** An abstraction of a UC Berkeley Teleop Slave Kinematics.  
 * Assumed to have the following setup:
 *      Encoder/Motor#    |        Function
 *           0            |       Pitch Tip
 *           1            |     Roll Clockwise
 *           2            |  Roll Counterclockwise
 *           3            |    Gross Rotation
 *           4            |    Right Base Motor (closest to supports)
 *           5            |   Forward Base Motor
 *           6            |     Left Base Motor
 *           7            |      Pneumatic Tip
 *
 *  Encoders corresponding from motors 0-3 are expected to output their values in radians
 *  where +volt = tighten, and encoders corresponding to motors 4-6 are expected to given
 *  their location in millimeters where +volt = extend.  7 is boolean.
 *
 *  Homing position (and therefore assumed startup position) is with motor 2 and gross 
 *  fully taut, all other rotations with no slack and at their minimum positions.  The 
 *  elbow  is assumed to be slack in the trocar, pushed as far backwards as possible 
 *  such that the arm is still as deep as it can be through it.  There is no bias 
 *  towards left and right.  [0;0;0] in tip coordinates originates at the center of 
 *  rotation of the arm in the trocar, where forward is +y and right is +x.  
 */
class Slave_mark1 : public Slave {
    public:
        double  pitch_min, pitch_max, roll_min, roll_max, gross_min, gross_max,
            arm_min, arm_max, cone_sphere_bound, radius_height_ratio, default_tension,
            cross_length, stick_length, boom_length, boom_altitude, link_length_with_pinion,
            link_altitude;

        /** The linear model with which we multiply goals
         *  after moving the x,y,z from the tip to the 
         *  elbow to obtain motor positions */
        CwMtx::CWTSquareMatrix<>* _wrist_to_motor_transform;
        CwMtx::CWTSquareMatrix<>* _elbow_to_base_transform;
        CwMtx::CWTSquareMatrix<>* _base_to_motor_transform;

//////////////////////////////////////////Initialization/////////////////////////////////////////
        /** Constructor, which expects num_actuator controllers, 
         * num_sensor sensors, and num_actuator actuators in an 
         * array corresponding to the spec above.  Also, the 
         * model (kinematics).  Initializes current slave 
         * pose as 0 on all channels */
        Slave_mark1(int id, boost::shared_ptr<Controller> controller, boost::shared_ptr<System> s);

        ~Slave_mark1();

        /** Reset the encoders on the slave such that its
         * current pose is the values given here.  Assume
         * (0,0,0) for (x,y,z) is the center of the trocar,
         * x being rightward, y being forward, z being 
         * upward. If use_base_coor isn't set, assumes
         * (x,y,z) is location of tip.  */
        void init_setpoints(std::map<int,double> point_value, bool use_base_coor=false);

        /** Helpers to above */
        void init_setpoints_arm(double pitch, double roll, double gross, double grip);
        void init_setpoints_base(double x, double y, double z, bool use_base_coor=false);

        /** Homes the slave into a reproducible position */
        void home();

        /** Parts of home() */
        void home_wrist();
        void home_base();


//////////////////////////////////////////Movement///////////////////////////////////////////////

        /** Returns a length 8 array of the motor positions
         * for each of the motors specified at the top of 
         * this file.  If use_base_coor isn't set, assumes
         * (x,y,z) is location of tip. */
        double* point_to_motors(double in[num_dof], double out[num_actuators], bool use_elbow_coor=false, bool use_kdl=false);
        double* elbow_to_base(double in[3], double out[3]);
        double* base_to_motor(double in[3], double out[3]);

        /** Inverse of point_to_motors If use_base_coor isn't set, assumes
         * (x,y,z) is location of tip unless use_base_coor. */
        double* motors_to_point(double motor_pos[num_actuators], double out[num_dof], bool use_base_coor=false, bool use_kdl=false);
        double* base_to_elbow(double in[3], double out[3]);
        double* motor_to_base(double in[3], double out[3]);
        
        /** non-linear portion of kinematics, from elbow to tip and back */
        double* elbow_tip_swap(double in[3], double out[3]);

        /** Copies the nearest legal pitch,roll,gross,x,y,z,grip into result */
        double* nearest_legal_point(double in[num_dof], double result[num_dof]);
};
#endif

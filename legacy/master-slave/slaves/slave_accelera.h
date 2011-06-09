#include "slave_mark1.h"
#include "accelera_geometry.h"

/** Identical base setup to the Slave_mark1.  Using new end effector,
* so wrist kinematics are slightly different */
class Slave_accelera : public Slave_mark1 { 
    public:
        Slave_accelera(int id, boost::shared_ptr<Controller> controller, boost::shared_ptr<System> sys);
        ~Slave_accelera();

        Accelera_geometry* _geometry;
        double* point_to_motors(double in[num_dof], double out[num_actuators], bool use_elbow_coor=false, bool use_kdl=false);
        double* motors_to_point(double motor_pos[num_actuators], double out[num_dof], bool use_base_coor=false, bool use_kdl=false);
        double* elbow_to_base(double in[3], double out[3]);
        double* base_to_motor(double in[3], double out[3]);
        double* elbow_tip_swap(double loc[3], double out[3]);
        double* base_to_elbow(double in[3], double out[3]);
        double* motor_to_base(double in[3], double out[3]);
        double* wrist_to_tip(double in[num_dof], double out[num_actuators]);
        double* point_to_tip(double in[num_dof], double out[num_dof], double* guess=NULL, bool called=false);
        double* tip_to_point(double in[num_dof], double out[num_dof]);
        double* CheckerboardToRobot(double in[num_dof], double out[num_dof]);
        
        void reset_wrists();
        void home();
};

/*********************************************
 **
 **
 **  DS0.h
 **
 **	DS0.h will describe the device/mechanism/DOF 
 ** data structures that will 
 ** 1) hold all data associated with a device and 
 ** 2) be "sampled" and passed to user-space for display and logging.
 **
 **    Devices contain mechanisms, mechanisms contain DOFs
 **
 **
 *********************************************/

#ifndef DS0_H
#define DS0_H
//#define NUM_MECH 2
#define MAX_DOF_PER_MECH 8
#define MAX_MECH_PER_DEV 4

#define STATE_OFF        0
#define STATE_UNINIT     1
#define STATE_READY      2
#define STATE_I_OVERLOAD 3

// for our coding convenience on ix86 platforms:
typedef int	            s_24;
typedef short int           s_16;
typedef unsigned char	    u_08;
typedef unsigned short int  u_16;
typedef unsigned int	    u_24;
typedef unsigned int	    u_32;
typedef unsigned long long int	    u_64;

/********************************************************
 *
 *  Structs for Cartesian values (formerly cartvals)
 *
 */
struct position
{
  int x;         // X coordinate
  int y;         // Y coordinate
  int z;         // Z coordiante
};

struct orientation
{
  float R[3][3];	// 3x3 Rotation Matrix (Dimensionless)
  int yaw;		// orientation expressed in XYZ Fixed frame notation
  int pitch;
  int roll;
  int grasp;
};

typedef enum
{
	TOOL_NONE,
	TOOL_GRASPER
} e_tool_type;


/*************************************************************************
 *
 *  Degree of Freedom Struct
 *      One of these for each mechanical Degree of Freedom
 *
 */
struct DOF {
  u_16 mech_type;
  u_16 type;
  int state;            // is this DoF enabled?
  s_24 enc_val;		// encoder value
//#ifdef WIN32_LEAN_AND_MEAN
  s_16 filler; // for 32 bit alignment for endianness
//#endif
  s_16 current_cmd;	// DAC command to achieve tau at actuator

  float jpos_off;	// offset to jpos to corrent for tool placement
  float jpos;		// actual DOF coordinate (rad)
  float mpos;

  float jvel; 		// actual DOF velocity(q-dot) 
  float mvel;
  float tau;		// actual DOF force/torque
  float tau_d;		// desired DOF force/torque 
  float tau_g;		// Estimated gravity force/torque on joint.
  float jpos_d;		// desired DOF coordinate (rad)
  float mpos_d;

  float jcon_jpos_d; // when in joint control - this is the intermediate desired joint position
  float jcon_jpos_t; // when in joint control - this is the target joint position

  float jpos_d_old;     // previous desired DOF coordinate (rad)
  float mpos_d_old;    
  float jvel_d;		// desired DOF velocity (q-dot-desired)
  float mvel_d;
  int enc_offset;       // Encoder offset to "zero"
  float perror_int;     // integrated position error for joint space position control

  float merr;		// Current motor error
  float merr_p;		// Previous motor error
  float merr_d;		// Error difference

};

/********************************************************
 *
 *  mechanism Struct
 *
 */
struct mechanism {
  u_16 type;
  s_16 filler1; // for 32 bit alignment for endianness

  struct position pos;
  struct position pos_d;
  struct position base_pos;     // base position in world frame
  struct orientation ori;
  struct orientation ori_d;
  struct orientation base_ori;  // base orientation in world frame

  struct position move_to_pos;  // target to move to while in joint_control

  struct DOF joint[MAX_DOF_PER_MECH];

  u_08 inputs;                  // input pins
  u_08 outputs;                 // output pins
  s_16 filler2; // for 32 bit alignment for endianness

  e_tool_type tool_type;

  int enabled; // This variable is set at startup (initialization). It's the manual enable/disable switch of the robots.
			   // If the robot is not enables - it's motors are inhibited and no motion can be produced.
  int homing_mode; // Homing mode
  int arm_homing_procedure_state;
  int tool_plate_placement_procedure_state;
  int tool_placement_procedure_state;
  int tool_homing_procedure_state;


  // not in use
  int /*long*/ processed_packets_from_master; // counter of processed packets from master
};

/********************************************************
 *
 *   device Struct
 *
 */
struct robot_device {
	unsigned int Ox12345678; // for checking endianness
  u_16 type;
  s_16 filler1; // for 32 bit alignment for endianness
  u_32 timestamp;	// time of last update
  u_08 runlevel;	// nothing/init/joints/kinematics/e-stop
  u_08 sublevel;	// which experimental mode are we running
  s_16 filler2; // for 32 bit alignment for endianness
  int  surgeon_mode;	// Clutching/indexing state - 1==engaged; 0==disengaged
  struct mechanism mech [MAX_MECH_PER_DEV];

  unsigned int ending_int; // for endiannness
};

#endif

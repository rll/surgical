/*********************************************
*
*
*  teleoperation.h
*
*    I define datastructures representing the 
*  information passed between master and slave 
*  in teleoperation.
*
*  Based on the wave variables naming schema:
*
*  u_struct passes from master to slave
*  v_struct passes from slave to master
*
*********************************************/

#ifndef TELEOPERATION_H
#define TELEOPERATION_H

#define SURGEON_ENGAGED 1
#define SURGEON_DISENGAGED 0
/*
u_struct : structure passed from master to slave.
This struct defines an incremental movment packet type.

sequence     Packet's sequence number
pactyp       protocol version in use 
version      Protocol version number  (***SRI) 

delx[2]	     position increment
dely[2]
delz[2]
delyaw[2]    Orientation increment
delpitch[2]
delroll[2]
buttonstate[2]
grasp[2]        +32767 = 100% closing torque, -32768 = 100% opening
surgeon_mode    SURGEON_ENGAGED or SURGEON_DISENGAGED  (formerly Pedal_Down or Pedal_UP)
checksum
*/

struct u_struct {
	unsigned int Ox12345678; // for checking endianness

	unsigned int sequence;
	unsigned int pactyp;  
	unsigned int version;

	unsigned int master_id; // each master should have his own id, all packets from same master should have same id

	int delx[2];
	int dely[2];
	int delz[2];
	int delyaw[2];
	int delpitch[2];
	int delroll[2];
	int buttonstate[2];
	int grasp[2];         

	int rotation_matrix[2][3][3];

	int surgeon_mode;     
	int checksum; 
	
};

/*
v_struct: Return DS from slave to master.
sequence
pactyp        protocol version in use
version       Protocol version number  (***SRI)
fx            X force
fy            Y force
fz            Z force
runlevel      Slave operating state
jointflags    bit flags for each joint limit (up to 16 joints).
checksum
*/
struct v_struct {
	int Ox12345678; // for checking endianness
	unsigned int sequence;
	unsigned int pactyp;
	unsigned int version;

	int initialized;

	int joint[2][7]; // robot joints, there are two robots, each 7 joints
	int rotation_matrix[2][3][3];

//	int fx;
//	int fy;
//	int fz;

	int runlevel;
	unsigned int  jointflags;
	int checksum;
};

#endif //teleoperation_h



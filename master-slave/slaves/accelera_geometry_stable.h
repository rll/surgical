#ifndef _ACCELERA_GEOMETRY_H_
#define _ACCELERA_GEOMETRY_H_

#include "shared.h"
#include "util.h"
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>


using namespace KDL; 
/** Includes all information about the kinematics of the accelera. Separate from an actual Slave_accelera so other places (such as controllers) can access it */
class Accelera_geometry{
    public:
    CwMtx::CWTSquareMatrix<>* _wrist_to_motor_transform;
    CwMtx::CWTSquareMatrix<>* _elbow_to_base_transform;
    CwMtx::CWTSquareMatrix<>* _base_to_motor_transform;
    CwMtx::CWTSquareMatrix<>* _gross_transform;
    CwMtx::CWTSquareMatrix<>* _pitch_transform;
    CwMtx::CWTSquareMatrix<>* _tilt_transform;
    CwMtx::CWTSquareMatrix<>* _tip_pitch_transform;
    CwMtx::CWTSquareMatrix<>* _tip_gross_transform;
    
    int id;
    double pitch_min;
    double pitch_max;
    double roll_min;
    double roll_max;
    double gross_min;
    double gross_max;
    double arm_min;
    double cone_sphere_bound;
    double arm_max;
    double radius_height_ratio;
    double default_tension;
    double stick_length;
    double cross_length;
    double boom_length;
    double boom_altitude;
    double link_length_with_pinion;
    double link_altitude;
    double wrist_gross_length;
    double wrist_pitch_length;
    double wrist_tilt_length;
    double angle_vertical_tip;
    double angle_rotation_tip;
    double test_kdl_arr[6];

    Accelera_geometry(int id);
    ~Accelera_geometry();
    void test_kdl( Frame F_dest);
    double* point_to_motors(const double in[num_dof], double out[num_actuators], bool use_elbow_coor=false);
    double* motors_to_point(const double motor_pos[num_actuators], double out[num_dof], bool use_elbow_coor=false);
    double* elbow_to_base(double in[3], double out[3]);
    double* base_to_motor(double in[3], double out[3]);
    double* elbow_tip_swap(double loc[3], double out[3], bool compute_vert=false, bool compute_rot=false);
    double* base_to_elbow(double in[3], double out[3], bool compute_vert=false, bool compute_rot=false);
    double* motor_to_base(double in[3], double out[3]);
    double* wrist_to_tip(double in[num_dof], double out[num_dof]);
    double* tip_to_wrist(double in[num_dof], double out[num_dof]);
    double* tip_as_point(double in[num_dof], double out[num_dof]);

};

#endif

#ifndef _try_energy_params_noniso_h
#define _try_energy_params_noniso_h

#include <stdlib.h>

#ifdef MAC
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <GL/gle.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h>
#endif

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include "../thread_discrete.h"
#include "../trajectory_reader.h"
#include "globals_thread_param_estimation.h"


#define ARG_PARAM_FILE 1
#define ARG_EVAL_MODE 2
#define ARG_MIN_TWIST 3
#define ARG_ITER_TWIST 4
#define ARG_MAX_TWIST 5
#define ARG_MIN_BEND_M1 6
#define ARG_ITER_BEND_M1 7
#define ARG_MAX_BEND_M1 8
#define ARG_MIN_BEND_M2 9
#define ARG_ITER_BEND_M2 10
#define ARG_MAX_BEND_M2 11
#define ARG_MIN_BEND_JOINT 12
#define ARG_ITER_BEND_JOINT 13
#define ARG_MAX_BEND_JOINT 14
#define ARG_NUM_MOVEMENTS 15


#define NUM_EXP 6
#define NUM_EXP_ANG 10
#define THREAD_MOVEMENT_STEPS 5

#define NUM_PARALLEL 8
#define RIBBON_HALF_WIDTH 3.35

enum ThreadEstimationModes {EVALGRAD, ONESTEPPROJECT, FULLMINIMIZATION};

ThreadEstimationModes estimationMode;



static char opt_info_dir[256];

static char data_write_base[256];
static char data_write_base_norm[256];

vector<Thread> all_threads;
vector<double> scores_each_thread;
vector<double> scores_each_thread_norm;
vector<double> scores_material_frame_thread;
vector<double> scores_twist_thread;
vector<double> scores_two_points_thread;
vector<double> scores_two_points_thread_norm;
vector<vector<Vector3d> > orig_points;
vector<vector<double> > orig_twist_angles;
vector<vector<Matrix3d> > orig_material_frames;

vector<Thread> construct_thread_sequence_forward(int startIndex, int numSeq, vector<Thread> &result);
double full_minimization(vector<double>& coeffs, int num_movements);


#endif

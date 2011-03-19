
#ifndef _try_energy_params_move_h
#define _try_energy_params_move_h

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
#define ARG_MIN_BEND 6
#define ARG_ITER_BEND 7
#define ARG_MAX_BEND 8
#define ARG_NUM_MOVEMENTS 9


#define NUM_EXP 6
#define NUM_EXP_ANG 10
#define NUM_PARALLEL 8
#define THREAD_MOVEMENT_STEPS 5

enum ThreadEstimationModes {EVALGRAD, ONESTEPPROJECT, FULLMINIMIZATION};

ThreadEstimationModes estimationMode;

static char opt_info_dir[256];
static char data_write_base[256];
static char data_write_base_norm[256];
vector<Thread> all_threads;
vector<double> scores_each_movement_norm;
vector<double> scores_each_thread_norm;
vector<double> scores_each_movement;
vector<double> scores_each_thread;
vector<vector<Vector3d> > orig_points;
vector<vector<double> > orig_twist_angles;


vector<Thread> construct_thread_sequence(int goalIndex, int numSeq, vector<Thread> &result); 
vector<Thread> construct_thread_sequence_forward(int startIndex, int numSeq, vector<Thread> &result); 
double full_minimization(vector<double>& coeffs, int num_movements);


#endif

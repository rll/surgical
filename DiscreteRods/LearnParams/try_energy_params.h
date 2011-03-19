
#ifndef _try_energy_params_h
#define _try_energy_params_h

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

#define NUM_PARALLEL 1

enum ThreadEstimationModes {EVALGRAD, ONESTEPPROJECT, FULLMINIMIZATION};

ThreadEstimationModes estimationMode;



static char opt_info_dir[256];
static char data_write_base[256];
vector<Thread> all_threads;
vector<double> scores_each_thread;
vector<double> angle_scores_each_thread;
vector<vector<Vector3d> > orig_points;
vector<vector<double> > orig_twist_angles;

double evalEnergyGradient(vector<double>& coeffs);
double oneStepDistance(vector<double>& coeffs);
double full_minimization(vector<double>& coeffs);


#endif

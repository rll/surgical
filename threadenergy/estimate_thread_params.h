#ifndef _estimate_thread_params_h
#define _estimate_thread_params_h

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
#include "thread_minenergy.h"
#include "trajectory_recorder.h"
#include "trajectory_reader.h"
#include "globals_thread_param_estimation.h"

#include "CompoundConstraint.h"
#include "BoundConstraint.h"

#define GRAV_FACTOR 1000000.0

#define NUM_MOTIONS_TO_SIM 0

struct optimization_info_thread_params
{
	NEWMAT::ColumnVector* orig_params;

	vector<Thread*> start_thread;
  vector<double> length_thread;;
  vector<vector<Thread_Motion>* > motions;
  vector<vector<savedThread>* > savedThreads;
  vector<vector<Thread*>* > threads;
	vector<int> num_pts_per;

	vector<NEWMAT::ColumnVector> savedParams;
  int f_eval_num;
	double bestVal;

  int f_eval_successful_num;
  vector<vector<double> > scoresPerThread;

	int num_motions_to_sim;
  int NUM_TRAJ;
  int num_threads_total;

};

static optimization_info_thread_params opt_params_thread_params; //will have to change this if we have multiple threads?


static char opt_info_dir[256];


void prepareOptimizationParams(char * basedir, char * numtraining_file);

void threadParamEvalFunction_init(int ndim, NEWMAT::ColumnVector& x);
void threadParamEvalFunction(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result);
void calculatePoints();
void evalEnergyParams(int argc, char * argv[]);
double medianOfVecs(vector<vector<double> >& scores);


#endif

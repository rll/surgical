#ifndef _estimate_thread_params_h
#define _estimate_thread_params_h

#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <math.h>
#include "thread_minenergy.h"
#include "trajectory_recorder.h"
#include "trajectory_reader.h"
#include "globals_thread_param_estimation.h"


struct optimization_info_thread_params
{
	NEWMAT::ColumnVector* orig_params; 

	Thread* start_thread;
	double length_thread;;
  vector<Thread_Motion> motions;
  vector<MatrixXd> points;
	int num_pts_per;
	

};

static optimization_info_thread_params opt_params_thread_params; //will have to change this if we have multiple threads?


void threadParamEvalFunction_init(int ndim, NEWMAT::ColumnVector& x);
void threadParamEvalFunction(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result);

#endif

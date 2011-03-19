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
#include "../thread_discrete.h"
#include "../threadpiece_discrete.h"
#include "../trajectory_reader.h"
#include "globals_thread_param_estimation.h"

#include "NLF.h"
#include "OptQNewton.h"
#include "NLP.h"
#include "GenSet.h"
#include "OptPDS.h"

//#include "CompoundConstraint.h"
//#include "BoundConstraint.h"

#define ARG_PARAMS_OUT_BASE 3
#define ARG_PARAM_FILE_IN 2
#define ARG_THREAD_FILE 1

#define NUM_PARALLEL 2

struct optimization_info_thread_params
{
	NEWMAT::ColumnVector* orig_params;
  vector<double> orig_coeffs;

	vector<Thread> all_threads;
  vector<vector<Vector3d> > orig_points;
  vector<vector<double> > orig_twist_angles;
  
	vector<NEWMAT::ColumnVector> savedParams;
  int f_eval_num;
	double bestVal;
};

static optimization_info_thread_params opt_params_thread_params; //will have to change this if we have multiple threads?


static char opt_info_dir[256];
static char data_write_base[256];


//void prepareOptimizationParams(char* thread_file) ;
void prepareOptimizationParams(char* thread_file);

void threadParamEvalFunction_init(int ndim, NEWMAT::ColumnVector& x);
void threadParamEvalFunction(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result);
//void calculatePoints();
//void evalEnergyParams(int argc, char * argv[]);
//double medianOfVecs(vector<vector<double> >& scores);


#endif

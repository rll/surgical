#ifndef _linearization_utils_h
#define _linearization_utils_h

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/Sparse>
#include <math.h>
#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/trajectory_reader.h"


#define WEIGHT_VERTICES 1.0
#define WEIGHT_EDGES 2.0

using namespace Eigen;

enum movement_mode {START, END, START_AND_END};

void applyControl(Thread* start, const VectorXd& u, const movement_mode movement = END);
void applyControl(Thread* start, const VectorXd& u, vector<Two_Motions*>& motions, const movement_mode movement = END);
void computeDifference(Thread* start, const Thread* goal, VectorXd& res);
void computeDifference_maxMag(Thread* start, const Thread* goal, VectorXd& res, double maxMag);
void solveLinearizedControl(Thread* start, const Thread* goal, const movement_mode movement = END);
void solveLinearizedControl(Thread* start, const Thread* goal, vector<Two_Motions*>& motions, const movement_mode movement = END);
void estimate_transition_matrix(Thread* thread, MatrixXd& A, const movement_mode movement = END);
void interpolateThreads(vector<Thread*>&traj, vector<Two_Motions*>& controls);
void simpleInterpolation(Thread* start, const Thread* goal, vector<Two_Motions*>& motions);



void iterative_control_opt(vector<Thread*>& trajectory, vector<VectorXd>& controls);
void ico_compute_massive_trans(vector<Thread*>& trajectory, SparseMatrix<double, RowMajor>& massive_mat);


#endif

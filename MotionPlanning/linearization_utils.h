#ifndef _linearization_utils_h
#define _linearization_utils_h

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <math.h>
#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/trajectory_reader.h"


#define WEIGHT_VERTICES 1.0
#define WEIGHT_EDGES 2.0
#define WEIGHT_ANGLE 1.0

enum movement_mode {START, END, START_AND_END};

void applyControl(Thread* start, const VectorXd& u, const movement_mode movement = START_AND_END);
void applyControl(Thread* start, const VectorXd& u, vector<Two_Motions*>& motions, const movement_mode movement = START_AND_END);
void computeDifference(Thread* start, const Thread* goal, VectorXd& res);
void computeDifference_maxMag(Thread* start, const Thread* goal, VectorXd& res, double maxMag);
void solveLinearizedControl(Thread* start, const Thread* goal, const movement_mode movement = START_AND_END);
void solveLinearizedControl(Thread* start, const Thread* goal, vector<Two_Motions*>& motions, const movement_mode movement = START_AND_END);
void estimate_transition_matrix(Thread* thread, MatrixXd& A, const movement_mode movement = START_AND_END);
void estimate_transition_matrix_noEdges_withTwist(Thread* thread, MatrixXd& A, const movement_mode movement = START_AND_END);
void interpolateThreads(vector<Thread*>&traj, vector<Two_Motions*>& controls);
void simpleInterpolation(Thread* start, const Thread* goal, vector<Two_Motions*>& motions);

#endif

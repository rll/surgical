#ifndef _linearization_utils_h
#define _linearization_utils_h

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <math.h>
#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/trajectory_reader.h"

void applyControl(Thread* start, const VectorXd& u);
void applyControl(Thread* start, const VectorXd& u, vector<Frame_Motion*>& motions);
void computeDifference(Thread* start, const Thread* goal, VectorXd& res);
void computeDifference_maxMag(Thread* start, const Thread* goal, VectorXd& res, double maxMag);
void solveLinearizedControl(Thread* start, const Thread* goal);
void solveLinearizedControl(Thread* start, const Thread* goal, vector<Frame_Motion*>& motions);
void estimate_transition_matrix(Thread* thread, MatrixXd& A);


#endif

#ifndef _linearization_utils_h
#define _linearization_utils_h

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <math.h>
#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/trajectory_reader.h"

void applyControl(Thread* start, const VectorXd& u);
void applyControl(Thread* start, const VectorXd& u, VectorXd* res);
void computeDifference(Thread* start, Thread* goal, VectorXd& res);
void computeDifference_maxMag(Thread* start, Thread* goal, VectorXd& res, double maxMag);
void solveLinearizedControl(Thread* start, Thread* goal);
void estimate_transition_matrix(Thread* thread, MatrixXd& A);


#endif

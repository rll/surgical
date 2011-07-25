#ifndef _linearization_utils_h
#define _linearization_utils_h

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <math.h>
#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/trajectory_reader.h"


#define WEIGHT_VERTICES 1.0
#define WEIGHT_EDGES 1.0
#define WEIGHT_ANGLE 1.0

using namespace Eigen;

enum movement_mode {START, END, START_AND_END};

void applyControl(Thread* start, const VectorXd& u, const movement_mode movement = START_AND_END);
//void applyControl(Thread* start, const VectorXd& u, vector<Two_Motions*>& motions, const movement_mode movement = START_AND_END);
void control_to_TwoMotion(const VectorXd& u, vector<Two_Motions*>& motions, const movement_mode movement = START_AND_END);
void TwoMotion_to_control(const Two_Motions* motion, VectorXd& u);
void TwoMotion_to_control(vector<Two_Motions*>& motions, VectorXd& u);
void computeDifference(Thread* start, const Thread* goal, VectorXd& res);
void computeDifference_maxMag(Thread* start, const Thread* goal, VectorXd& res, double maxMag);
void solveLinearizedControl(Thread* start, const Thread* goal, const movement_mode movement = START_AND_END);
void solveLinearizedControl(Thread* start, const Thread* goal, VectorXd& motions, const movement_mode movement = START_AND_END);
void estimate_transition_matrix(Thread* thread, MatrixXd& A, const movement_mode movement = START_AND_END);
void estimate_transition_matrix_noEdges_withTwist(Thread* thread, MatrixXd& A, const movement_mode movement = START_AND_END);
void estimate_transition_matrix_withTwist(Thread* thread, MatrixXd& A, const movement_mode movement = START_AND_END);
void interpolateThreads(vector<Thread*>&traj, vector<VectorXd>& controls);
void simpleInterpolation(Thread* start, const Thread* goal, VectorXd& control);

#endif

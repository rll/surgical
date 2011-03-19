#ifndef _thread_RRT_h
#define _thread_RRT_h

#include "thread_discrete.h"
#include "threadutils_discrete.h"
#include <Eigen/Geometry>
#include <vector>

using namespace std;

class RRTNode {
 public:
  ~RRTNode();
  RRTNode();
  RRTNode(const Thread* start);

  RRTNode* next;
  RRTNode* prev;
  vector<Frame_Motion*> lstMotions;
  MatrixXd B;
  bool linearized;

  VectorXd x;
  VectorXd twists;
  Matrix3d endrot;
  int N;

  Matrix3d endRotation() {  return endrot; }
  Vector3d endPosition() {  return x.segment<3>(N-3); }
};

class Thread_RRT
{
 public:
  Thread_RRT();
  ~Thread_RRT();

  void planPath(const Thread* start, const Thread* goal, vector<Frame_Motion>& movements);
  void initialize(const Thread* start, const Thread* goal);
  void planStep(VectorXd* goal, VectorXd* prev, VectorXd* next_thread);
  vector<RRTNode*>* getTree() { return &_tree; }

 private:
  vector<RRTNode*> _tree;
  VectorXd _goal;
  Matrix3d _goal_rot;

  void getNextGoal(VectorXd* next, Matrix3d* next_rot);
  double extendToward(const VectorXd& next, const Matrix3d& next_rot);
  double largeRotation(const VectorXd& next);
  RRTNode* findClosestNode(const VectorXd& next);
  void applyControl(Thread* start, const VectorXd& u, VectorXd* res, Frame_Motion* motion);


  void simpleInterpolation(const Vector3d& cur_pos, const Matrix3d& cur_rot, const Vector3d& next, const Matrix3d& next_rot, Vector3d* res_translation, Matrix3d* res_rotation);


  double distToGoal;
  double bestDist;
  double TOLERANCE;
  VectorXd next;
  Matrix3d next_rot;
};

#endif

#ifndef _thread_RRT_h
#define _thread_RRT_h

#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/threadutils_discrete.h"
#include "linearization_utils.h"
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

//  VectorXd x;
//  VectorXd twists;
//  Matrix3d endrot;
  Thread thread;
  int N;

  double CVF; 

  Matrix3d endRotation() {  return thread.end_rot(); }
  //Matrix3d endRotation() {  return endrot; }
  Vector3d endPosition() {  return thread.end_pos(); }
  //Vector3d endPosition() {  return x.segment<3>(N-3); }
  const Thread getThread() { return thread; } 
};

class Thread_RRT
{
 public:
  Thread_RRT();
  ~Thread_RRT();

  //void planPath(const Thread* start, const Thread* goal, vector<Frame_Motion>& movements);
  void initialize(const Thread* start, const Thread* goal);
  //void planStep(VectorXd* goal, VectorXd* prev, VectorXd* next_thread);
  void planStep(Thread& new_sample_thread, Thread& closest_sample_thread, Thread& new_extend_thread);
  vector<RRTNode*>* getTree() { return &_tree; }
  Thread* generateSample(const Thread* start);
 
 
 private:
  vector<RRTNode*> _tree;
  VectorXd _goal;
  Matrix3d _goal_rot;
  const Thread* _start_thread;
  const Thread* _goal_thread;

//  void getNextGoal(VectorXd* next, Matrix3d* next_rot);
  void getNextGoal(Thread* next);
  //double extendToward(const VectorXd& next, const Matrix3d& next_rot);
  double extendToward(const Thread* target);
  //double largeRotation(const VectorXd& next);
  double largeRotation(const Thread* target);
  //RRTNode* findClosestNode(const VectorXd& next);
  RRTNode* findClosestNode(const Thread* target);

  double distanceBetween(const Thread* start, const Thread* end); 

  void simpleInterpolation(const Vector3d& cur_pos, const Matrix3d& cur_rot, const Vector3d& next, const Matrix3d& next_rot, Vector3d* res_translation, Matrix3d* res_rotation);


  double distToGoal;
  double bestDist;
  double TOLERANCE;
//  VectorXd next;
//  Matrix3d next_rot;
  Thread* next_thread;

};

#endif

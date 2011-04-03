#ifndef _rrt_utils_h
#define _rrt_utils_h

#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/threadutils_discrete.h"
#include "linearization_utils.h"
#include <Eigen/Geometry>
#include <vector>
#include <set>

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
  Thread* thread;
  int N;

  double CVF; 
  float* data;

  const Matrix3d &endRotation() {  return thread->end_rot(); }
  //Matrix3d endRotation() {  return endrot; }
  const Vector3d &endPosition() {  return thread->end_pos(); }
  //Vector3d endPosition() {  return x.segment<3>(N-3); }
  const Thread* getThread() { return thread; } 
  const float* getData() { return data; }

};


class RRTNodeUtils {

  public:
    double distanceBetween(RRTNode* start, RRTNode* end);

};

#endif

#ifndef _thread_RRT_h
#define _thread_RRT_h

#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/threadutils_discrete.h"
#include "linearization_utils.h"
#include "rrt_utils.h" 
#include <lshkit.h> 
#include "lsh-table.h" 
#include <Eigen/Geometry>
#include <vector>
#include <set>

using namespace std;
using namespace lshkit; 

class Thread_RRT
{
 public:

  //typedef RRTNode* Key;
  //typedef float* Value;
  //typedef float* Domain;
  //typedef RRTNode::Accessor ACCESSOR;
  //typedef lshkit::metric::l2<float> METRIC;
  //typedef lshkit::HyperPlaneLsh LSH;

  Thread_RRT();
  ~Thread_RRT();

  //void planPath(const Thread* start, const Thread* goal, vector<Frame_Motion>& movements);
  void initialize(const Thread* start, const Thread* goal);
  //void planStep(VectorXd* goal, VectorXd* prev, VectorXd* next_thread);
  void planStep(Thread& new_sample_thread, Thread& closest_sample_thread, Thread& new_extend_thread);
  vector<RRTNode*>* getTree() { return &_tree; }
  Thread* generateSample(const Thread* start);
  
  typedef Repeat<HyperPlaneLsh> HASH; 
 
 private:
  vector<RRTNode*> _tree;
  VectorXd _goal;
  Matrix3d _goal_rot;
  const Thread* _start_thread;
  const Thread* _goal_thread;


  void insertIntoRRT(RRTNode* node);  
//  void getNextGoal(VectorXd* next, Matrix3d* next_rot);
  void getNextGoal(Thread* next);
  //double extendToward(const VectorXd& next, const Matrix3d& next_rot);
  double extendToward(const Thread* target);
  double extendAsFarToward(const Thread* target);
  //double largeRotation(const VectorXd& next);
  double largeRotation(const Thread* target);
  //RRTNode* findClosestNode(const VectorXd& next);
  RRTNode* findClosestNode(const Thread* target);
  double distanceBetween(const Thread* start, const Thread* end); 

//  void simpleInterpolation(const Vector3d& cur_pos, const Matrix3d& cur_rot, const Vector3d& next, const Matrix3d& next_rot, Vector3d* res_translation, Matrix3d* res_rotation);
  void simpleInterpolation(const Thread* start, const Thread* end, Vector3d* res_translation, Matrix3d* res_rotation);

  double distToGoal;
  double bestDist;
  double TOLERANCE;
  double totalPenalty; 
//  VectorXd next;
//  Matrix3d next_rot;
  Thread* next_thread;
  //lshkit::LshIndex<lshkit::HyperPlaneLsh, RRTNode* > *index; 
  LshTable<HASH>*index;  
  
};

#endif

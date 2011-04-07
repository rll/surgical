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

  Thread_RRT();
  ~Thread_RRT();

  //void planPath(const Thread* start, const Thread* goal, vector<Two_Motions>& movements);
  void initialize(const Thread* start, const Thread* goal);
  void planStep(Thread& new_sample_thread, Thread& closest_sample_thread, Thread& new_extend_thread);
  void updateBestPath(); 
  vector<RRTNode*>* getTree() { return &_tree; }
  Thread* generateSample(const Thread* start);
  Thread* generateSample(int N); 

  double distanceBetween(const Thread* start, const Thread* end) {
    RRTNode* sNode = new RRTNode(start);
    RRTNode* eNode = new RRTNode(end);
    double score = utils.distanceBetween(sNode, eNode); 
    delete sNode; 
    delete eNode; 
    return score;
  }

  double l2PointsDifference(const Thread* start, const Thread* end) { 
    RRTNode* sNode = new RRTNode(start);
    RRTNode* eNode = new RRTNode(end);
    double score = utils.l2PointsDifference(sNode, eNode); 
    delete sNode; 
    delete eNode; 
    return score;
  }

  RRTNode* findClosestNode(const Thread* target, bool approximateNode=true);
  typedef Repeat<HyperPlaneLsh> HASH; 

  Thread* halfDimApproximation(const Thread* target);
  Thread* doubleDimApproximation(const Thread* target); 

 private:
  vector<RRTNode*> _tree;
  const RRTNode* _start_node; 
  const RRTNode* _goal_node;

  void insertIntoRRT(RRTNode* node);  
  Thread* getNextGoal();
  double extendToward(Thread* target);
  double extendAsFarToward(Thread* target);
  double largeRotation(const Thread* target);



  void simpleInterpolation(Thread* start, const Thread* end, vector<Two_Motions*>& motions);

  double distToGoal;
  double bestDist;
  double TOLERANCE;
  double totalPenalty; 
  Thread* next_thread;
  LshMultiTable<HASH>*index; 

  RRTNodeUtils utils; 
  
};

#endif

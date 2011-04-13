#ifndef _planner_lib_h_
#define _planner_lib_h_

#define NUM_INTERPOLATION 100
#define NUM_NODES 30000
#define RRT_L2_POINTS_THRESHOLD 2.0
#define NUM_ITERS_SQP 10
#define SUBSAMPLE_TO_THIS_NUMBER 100

#include "planner_utils.h"
#include "linearization_utils.h"
#include "trajectory_follower.h" 
#include "iterative_control.h"
#include <boost/progress.hpp> 
#include <sys/time.h>

Thread_RRT util_planner; 

/* 
 * Interpolates points and start/end constraints using quaternion interpolation
 */
void interpolatePointsTrajectory(Thread* start, Thread* end, vector<Thread*>& traj) 
{
  Thread* start_copy = new Thread(*start);
  Thread* end_copy = new Thread(*end); 
  // wrap threads and controls
  traj.resize(NUM_INTERPOLATION);
  traj[0] = start_copy;
  traj[NUM_INTERPOLATION-1] = end_copy;
  vector<VectorXd> controls;

  //call interpolate threads and put results in traj 
  interpolateThreads(traj, controls);
};

/*
 * Interpolates only start/end constraints using quaternion interpolation
 */ 

void interpolateEndsTrajectory(Thread* start, Thread* end, vector<Thread*>& traj)
{
  assert("Not implemented yet"); 
};


/*
 * Open loop controller applies controls in controls_in at every time step
 * Assumes start thread is traj_in[0]
 */
void openLoopController(vector<Thread*>& traj_in, vector<VectorXd>& controls_in, vector<Thread*>& traj_out) {
  Thread* thread = new Thread(*traj_in[0]);
  for (int i = 0; i < controls_in.size(); i++) {
    traj_out.push_back(new Thread(*thread));
    applyControl(thread, controls_in[i]);
  }
  traj_out.push_back(new Thread(*thread));
}

/* 
 * Closed loop controller implemented via Trajectory_Follower
 * Assumes start thread is in traj_in[0]
 */
void closedLoopLinearizationController(vector<Thread*>& traj_in, vector<vector<VectorXd> >& controls_in, vector<Thread*>& traj_out)
{
  // copy input trajectory 
  vector<Thread*> traj_in_copy;
  traj_in_copy.resize(traj_in.size()-1); 
  for (int i = 1; i < traj_in.size(); i++) {
    traj_in_copy[i-1] = new Thread(*traj_in[i]);
  }

  Thread* start_copy = new Thread(*traj_in[0]);

  // follow using trajectory follower
  Trajectory_Follower *follower = 
    new Trajectory_Follower(traj_in_copy, controls_in, start_copy);
  follower->control_to_finish(); 

  //put states reached in traj_out 
  follower->getReachedStates(traj_out); 
};
/* 
 * Follow the trajectory in traj_in towards goal.
 * Start is assumed to be traj_in[0]
 * This is implemented with Trajectory_Follower
 */
void linearizeViaTrajectory(vector<Thread*>& traj_in, vector<Thread*>& traj_out){
  // wrap controls as 0
  vector<vector<VectorXd> > controls_in;
  for (int i = 0; i < traj_in.size()-1; i++) {
    VectorXd zero_control(12);
    zero_control.setZero();
    vector<VectorXd> wrapper_zero_control;
    wrapper_zero_control.push_back(zero_control);
    controls_in.push_back(wrapper_zero_control); 
  }
  closedLoopLinearizationController(traj_in, controls_in, traj_out);
};

/* 
 * Given start, linearize towards end as much as possible
 * This is implemented with Trajectory_Follower
 */
void linearizeToGoal(Thread* start, Thread* end, vector<Thread*>& traj) 
{

  // wrap trajectory
  Thread* start_copy = new Thread(*start);
  Thread* end_copy = new Thread(*end); 

  vector<Thread*> traj_in;
  traj_in.push_back(start_copy); 
  traj_in.push_back(end_copy);

  // call linearize via trajectory (containing only start and end) and put 
  // result in traj 
  linearizeViaTrajectory(traj_in, traj); 
};

/*
 * Use SQP solver given traj_in. Puts results in traj_out and control_out
 */
void solveSQP(vector<Thread*>& traj_in, vector<Thread*>& traj_out, vector<VectorXd>& control_out, const char* namestring)
{
  int num_iters = NUM_ITERS_SQP; 
  
  // Wrap controls and put threads in traj_out as copies 
  traj_out.resize(traj_in.size());
  for (int i = 0; i < traj_in.size(); i++) {
    traj_out[i] = new Thread(*traj_in[i]);
  }
  
  Iterative_Control* ic = 
    new Iterative_Control(traj_out.size(), traj_out[0]->num_pieces());
  ic->set_namestring(namestring);
  ic->iterative_control_opt(traj_out, control_out, num_iters);

};

/* 
 * Build an RRT from start towards end, using num_dim_reduc dimension reductions. 
 */
void RRTPlanner(Thread* start, Thread* end, int num_dim_reduc, vector<Thread*>& traj, vector<vector<VectorXd> >& mot) 
{  
  Thread_RRT planner;  
  Thread* start_copy = new Thread(*start);
  Thread* end_copy = new Thread(*end); 

  if (num_dim_reduc == 0) {
    planner.initialize(new Thread(*start_copy), new Thread(*end_copy));

    Thread goal_thread; Thread prev_thread; Thread next_thread;
    double bestScore = DBL_MAX;
    while(planner.getTree()->size() < NUM_NODES && 
        bestScore > RRT_L2_POINTS_THRESHOLD) {
      //#pragma omp parallel for num_threads(NUM_CPU_THREADS)
      //for (int i = 0; i < 24; i++) {
        planner.planStep(goal_thread, prev_thread, next_thread);
      //}

      planner.updateBestPath();
      
      RRTNode* node = planner.getTree()->front();
      while (node->next != NULL) {
        node = node->next;
      }
      bestScore = planner.l2PointsDifference(node->thread, end); 
      
    }

    RRTNode* node = planner.getTree()->front();
    node = node->next;
    while (node != NULL) {
      traj.push_back(new Thread(*node->thread));
      vector<VectorXd> wrapper_mot;
      wrapper_mot.push_back(node->motion);
      mot.push_back(wrapper_mot);
      node = node->next;
    }

  } else { 
    Thread* approxStart = planner.halfDimApproximation(start_copy);
    Thread* approxTarget = planner.halfDimApproximation(end_copy); 
    vector<Thread*> path;
    vector<vector<VectorXd> > motions; 
    RRTPlanner(approxStart, approxTarget, num_dim_reduc-1, path, motions);

    // transform the path to higher D space 
    vector<Thread*> transformed_path; 
    transformed_path.resize(path.size());
    boost::progress_display progress(path.size()); 
    for (int i = 0; i < path.size(); i++) {
      transformed_path[i] = planner.doubleDimApproximation(path[i]);
      ++progress; 
    }

    // follow the trajectory of the transformed approximation
    Trajectory_Follower *pathFollower = 
      new Trajectory_Follower(transformed_path, motions, start_copy);

    pathFollower->control_to_finish();
    pathFollower->getReachedStates(traj);
    pathFollower->getMotions(mot);
  }
};

/*
 * Subsample traj according to some method
 */
void traj_subsampling(vector<Thread*>& traj_in, vector<Thread*>& traj_out) {
  
  int rate = (int)(traj_in.size())/SUBSAMPLE_TO_THIS_NUMBER;

  for (int i = 0; i < traj_in.size(); i++) {
    if (i % rate == 0) { 
      traj_out.push_back(new Thread(*traj_in[i]));
    }
  }
};

double cost_metric(Thread* u, Thread* v) {
  return util_planner.l2PointsDifference(u,v);
}

class Timer {
  public: 
    Timer() {
      gettimeofday(&start_tv, NULL); 
    }
    void restart() {
      gettimeofday(&start_tv, NULL);
    }
    double elapsed() {
      gettimeofday(&tv, NULL); 
      return  (tv.tv_sec - start_tv.tv_sec) +
        (tv.tv_usec - start_tv.tv_usec) / 1000000.0;
    }

  private:
    struct timeval tv;
    struct timeval start_tv;

};



#endif

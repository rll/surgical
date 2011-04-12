#define NUM_INTERPOLATION 100

#include "linearization_utils.h"
#include "trajectory_follower.h" 
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
  vector<Two_Motions*> controls;

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
 * Given a start and end, follow the trajectory in traj_in towards goal
 * This is implemented with Trajectory_Follower
 */
void linearizeViaTrajectory(vector<Thread*>& traj_in, vector<Thread*>& traj_out){
  // wrap controls
  vector<vector<Two_Motions*> > controls_in;
  for (int i = 0; i < traj_in.size(); i++) {
    VectorXd zero_control(12);
    zero_control.setZero(); 
    vector<Two_Motions*> tm_zero_control;
    control_to_TwoMotion(zero_control, tm_zero_control);
    controls_in.push_back(tm_zero_control); 
  }
  Thread* start_copy = new Thread(*traj_in[0]);

  // follow using trajectory follower
  Trajectory_Follower *follower = 
    new Trajectory_Follower(traj_in, controls_in, start_copy);
  follower->control_to_finish(); 

  //put states reached in traj_out 
  follower->getReachedStates(traj_out); 
};

/* 
 * Given a start and end thread, linearize to goal until no improvement.
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
void solveSQP(vector<Thread*>& traj_in, vector<Thread*>& traj_out, vector<VectorXd>& control_out)
{
  assert("Not implemented yet"); 
};

/* 
 * Use SQP trajectory and controls (apply controls). Start is assumed to be traj_in[0]   
 */
void openLoopSQP(vector<Thread*>& traj_in, vector<VectorXd>& control_in, vector<Thread*>& traj_out)
{
  assert("Not implemented yet"); 
};


/* 
 * Use SQP trajectory and controls, but use linearization to do control. Start is assumed to be in traj_in[0]
 */
void closedLoopSQP(vector<Thread*>& traj_in, vector<VectorXd>& control_in, vector<Thread*>& traj_out) 
{ 
  assert("Not implemented yet"); 
};

/* 
 * Build an RRT from start towards end, using num_dim_reduc dimension reductions. 
 */
void RRTPlanner(Thread* start, Thread* end, int num_dim_reduc, vector<Thread*>& traj, vector<VectorXd>& mot) 
{  
  assert("Not implemented yet"); 
};

/*
 * Subsample traj according to some method
 */
void traj_subsampling(vector<Thread*>& traj_in, vector<Thread*>& traj_out) { 
  assert("Not implemented yet"); 
};




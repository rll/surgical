#define NUM_INTERPOLATION 100

#include "linearization_utils.h"

/* 
 * Interpolates points and start/end constraints using quaternion interpolation
 */
void interpolatePointsTrajectory(Thread* start, Thread* end, vector<Thread*>& traj) 
{
  Thread* start_copy = new Thread(*start);
  Thread* end_copy = new Thread(*end); 
  traj.resize(NUM_INTERPOLATION);
  traj[0] = start_copy;
  traj[NUM_INTERPOLATION-1] = end_copy;
  vector<Two_Motions*> controls;
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
 * Given a start and end thread, linearize to goal until no improvement.
 * This is implemented under Trajectory_Follower
 */
void linearizeToGoal(Thread* start, Thread* end, vector<Thread*>& traj) 
{
  assert("Not implemented yet"); 
};


/* 
 * Given a start and end, follow the trajectory in traj_in towards goal
 * This is implemented under Trajectory_Follower
 */
void linearizeViaTrajectory(Thread* start, Thread* end, vector<Thread*>& traj_in, vector<Thread*>& traj_out) 
{
  assert("Not implemented yet"); 
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




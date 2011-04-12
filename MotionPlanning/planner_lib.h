
void interpolatePointsTrajectory(Thread* start, Thread* end, vector<Thread*>& traj);
void interpolateEndsTrajectory(Thread* start, Thread* end, vector<Thread*>& traj);


// all this do are "following" 
void linearizeToGoal(Thread* start, Thread* end, vector<Thread*>& traj);
void linearizeViaTrajectory(Thread* start, Thread* end, vector<Thread*>& traj_in, vector<Thread*>& traj_out);

void solveSQP(vector<Thread*>& traj_in, vector<Thread*>& traj_out, vector<VectorXd>& control_out);

// all this does is following
void openLoopSQP(vector<Thread*>& traj_in, vector<VectorXd>& control_in, vector<Thread*>& traj_out);
void closedLoopSQP(vector<Thread*>& traj_in, vector<VectorXd>& control_in, vector<Thread*>& traj_out);

void RRTPlanner(Thread* start, Thread* end, int num_dim_reduc, vector<Thread*>& traj, vector<VectorXd>& mot);


void RRT_Subsampling(vector<Thread*>& traj_in, vector<Thread*>& traj_out); 





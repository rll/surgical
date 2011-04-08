#include "trajectory_follower.h"

Trajectory_Follower::Trajectory_Follower(vector<Thread*>& trajectory, vector<vector<Two_Motions*> >& motions, Thread* start_thread) :
  _curr_ind(0)
{
  _trajectory = trajectory;
  _motions = motions;

  _reached_states.resize(1);
  _reached_states[0] = new Thread(*start_thread);
}

Trajectory_Follower::~Trajectory_Follower()
{
}


vector<Two_Motions*> Trajectory_Follower::Take_Step(int max_linearizations)
{
  vector<Two_Motions*> motionsGenerated;
  if (is_done())
    return motionsGenerated;
  Thread* next_state = new Thread(*_reached_states.back());

  const double linearization_error_thresh = 1;

  //next_state->apply_motion(*_motions[_curr_ind]);
  vector<Two_Motions*> motionLst = _motions[_curr_ind];
  for (int i = 0; i < motionLst.size(); i++) { 
    next_state->apply_motion_nearEnds(*motionLst[i]);
  }
  _curr_ind++;

  double error_last_linearization = calculate_thread_error(_trajectory[_curr_ind], next_state);
  for (int linearization_num=0; linearization_num < max_linearizations; linearization_num++)
  {
    vector<Two_Motions*> tmpMotions;
    solveLinearizedControl(next_state, _trajectory[_curr_ind], tmpMotions, START_AND_END); 
    for (int i = 0; i < tmpMotions.size(); i++) {
      motionsGenerated.push_back(tmpMotions[i]);
    }
    double error_this_linearizaton = calculate_thread_error(_trajectory[_curr_ind], next_state);
    if (error_this_linearizaton + linearization_error_thresh > error_last_linearization) 
      break;

    error_last_linearization = error_this_linearizaton;
  } 

  cout << "adding to reached states " << next_state << endl; 
  _reached_states.push_back(next_state);
  cout << "reached states size: " << _reached_states.size() << endl ; 
  return motionsGenerated; 
}

bool Trajectory_Follower::is_done()
{
  return _curr_ind >= (_trajectory.size()-1);

}


double Trajectory_Follower::calculate_thread_error(Thread* start, Thread* goal)
{
  vector<Vector3d> points_start;
  vector<double> angles_start;
  start->get_thread_data(points_start, angles_start);
  vector<Vector3d> points_goal;
  vector<double> angles_goal;
  goal->get_thread_data(points_goal, angles_goal);
  
  return calculate_vector_diff_norm(points_start, points_goal);
}


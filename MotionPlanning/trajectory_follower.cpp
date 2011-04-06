#include "trajectory_follower.h"

Trajectory_Follower::Trajectory_Follower(vector<Thread*>& trajectory, vector<Two_Motions*>& motions, Thread* start_thread) :
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


void Trajectory_Follower::Take_Step(int max_linearizations)
{
  if (is_done())
    return;
  Thread* next_state = new Thread(*_reached_states.back());

  const double linearization_error_thresh = 1.0;

  next_state->apply_motion(*_motions[_curr_ind]);
  _curr_ind++;

  double error_last_linearization = calculate_thread_error(_trajectory[_curr_ind], next_state);
  for (int linearization_num=0; linearization_num < max_linearizations; linearization_num++)
  {
    solveLinearizedControl(_trajectory[_curr_ind], next_state);
    double error_this_linearizaton = calculate_thread_error(_trajectory[_curr_ind], next_state);
    if (error_this_linearizaton + linearization_error_thresh > error_last_linearization)
      break;

    error_last_linearization = error_this_linearizaton;
  } 

  _reached_states.push_back(next_state);
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


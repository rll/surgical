#include "trajectory_follower.h"
#include <boost/progress.hpp>


Trajectory_Follower::Trajectory_Follower(vector<Thread*>& trajectory, vector<vector<VectorXd> >& motions, Thread* start_thread) :
  _curr_ind(0)
{
  _trajectory = trajectory;
  //_motions = motions;
	for (int i=0; i < motions.size(); i++)
	{
		vector<VectorXd> copy_vec_motions;
		for (int j=0; j < motions[i].size(); j++)
		{
			copy_vec_motions.push_back(motions[i][j]);
		}
		_motions.push_back(copy_vec_motions);
	}

  _reached_states.resize(1);
  _reached_states[0] = new Thread(*start_thread);
	VectorXd zero_control(12);
	zero_control.setZero();
	vector<VectorXd> wrapper_zero_control;
	wrapper_zero_control.push_back(zero_control);
	_reached_states_motions.push_back(wrapper_zero_control); 


}

Trajectory_Follower::~Trajectory_Follower()
{
}

void Trajectory_Follower::Take_Step()
{
  vector<VectorXd> motionsGenerated;
  Thread* next_state = new Thread(*_reached_states.back());

  const double linearization_error_thresh = 1e-4;
  const int count_no_improvement_thresh = 1; 
  int count_no_improvement = 0; 

  vector<VectorXd> motionLst = _motions[_curr_ind];
  for (int i = 0; i < motionLst.size(); i++) { 
		applyControl(next_state, motionLst[i]);
  }
  _curr_ind++;



  double error_last_linearization = calculate_thread_error(_trajectory[_curr_ind], next_state);
  //for (int linearization_num=0; linearization_num < max_linearizations; linearization_num++)
  do {
    VectorXd tmpMotion;
    solveLinearizedControl(next_state, _trajectory[_curr_ind], tmpMotion, START_AND_END); 
    double error_this_linearizaton = calculate_thread_error(_trajectory[_curr_ind], next_state);
    if (error_this_linearizaton + linearization_error_thresh > error_last_linearization) {  
      //next_state = prevThread; 
      //break;
      count_no_improvement += 1; // this count should never be reset
    } 

		motionsGenerated.push_back(tmpMotion);

    error_last_linearization = error_this_linearizaton;
  } while (count_no_improvement < count_no_improvement_thresh); 

  //cout << "adding to reached states " << next_state << endl; 
  _reached_states.push_back(next_state);
  _reached_states_motions.push_back(motionsGenerated);
  
}

bool Trajectory_Follower::is_done()
{
  return _curr_ind >= (_trajectory.size()-1);

}

void Trajectory_Follower::control_to_finish() {
  cout << "Control to finish" << endl; 
  boost::progress_display progress(_trajectory.size() - _curr_ind - 1); 

  while(!is_done()) { 
    Take_Step();
    ++progress;
  }
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

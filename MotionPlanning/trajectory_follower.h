#ifndef _trajectory_follower_h
#define _trajectory_follower_h

#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/trajectory_reader.h"
#include "linearization_utils.h"




class Trajectory_Follower
{
  public:
    Trajectory_Follower(vector<Thread*>& trajectory, vector<vector<VectorXd> >& motions, Thread* start_thread);

    ~Trajectory_Follower();


    void Take_Step();
    bool is_done();

    Thread* curr_state() {return _reached_states.back();}
    Thread* curr_target() { 
      if (!is_done()) { 
        return _trajectory[_curr_ind]; 
      }
    }
    const int curr_ind() const {return _curr_ind;}
    void getReachedStates(vector<Thread*>& traj) {
      traj.resize(0);
      for (int i = 0; i < _reached_states.size(); i++) {
        traj.push_back(_reached_states[i]);
      }
    }

    void getMotions(vector<vector<VectorXd> >& motions) {
      motions.resize(0);
      for (int i = 0; i < _reached_states_motions.size(); i++) {
        motions.push_back(_reached_states_motions[i]);
      }
    }


    void control_to_finish();

  protected:
    vector<Thread*> _trajectory;
    vector<Thread*> _reached_states;
    vector<vector<VectorXd> > _reached_states_motions; 
    vector<vector<VectorXd> > _motions;
    int _curr_ind;


    double calculate_thread_error(Thread* start, Thread* goal);

};



#endif

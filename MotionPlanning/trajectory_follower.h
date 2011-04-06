#ifndef _trajectory_follower_h
#define _trajectory_follower_h

#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/trajectory_reader.h"
#include "linearization_utils.h"




class Trajectory_Follower
{
  public:
    Trajectory_Follower(vector<Thread*>& trajectory, vector<Two_Motions*>& motions, Thread* start_thread);

    ~Trajectory_Follower();


    void Take_Step(const int max_linearization = 1);
    bool is_done();

    Thread* curr_state() {return _reached_states.back();}
    const int curr_ind() const {return _curr_ind;}



  protected:
    vector<Thread*> _trajectory;
    vector<Thread*> _reached_states;
    vector<Two_Motions*> _motions;
    int _curr_ind;


    double calculate_thread_error(Thread* start, Thread* goal);
    



};



#endif

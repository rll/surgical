#define NUM_ITERS_SQP_PLANNER 1
#define NUM_ITERS_SQP_SMOOTHER 1
#define SQP_BREAK_THRESHOLD 3.5

#include "../MotionPlanning/worldSQP.h"
#include "IO/Control.h"
#include <iostream>
#include <sys/time.h>
#include <boost/progress.hpp>



double l2PointsDifference(VectorXd& a, VectorXd& b) {
  return (a-b).norm();
}

double l2PointsDifference(World* a, World* b) { 
  VectorXd state_a, state_b;
  a->getStateForJacobian(state_a);
  b->getStateForJacobian(state_b);
  return l2PointsDifference(state_a, state_b); 
}


double cost_metric(World* a, World* b) { 
  return l2PointsDifference(a,b);
}

/*
 * Use SQP solver given traj_in. Puts results in traj_out and control_out
 */
void solveSQP(vector<World*>& traj_in, vector<World*>& traj_out, vector<VectorXd>& control_out, vector<vector<World*> >& sqp_debug_data, const char* namestring, bool planner = true)
{
  int num_iters;
  if (planner) {
    num_iters = NUM_ITERS_SQP_PLANNER;
  } else {
    num_iters = NUM_ITERS_SQP_SMOOTHER;
  }

  // Wrap controls and put threads in traj_out as copies 
  traj_out.resize(traj_in.size());
  for (int i = 0; i < traj_in.size(); i++) {
    traj_out[i] = new World(*traj_in[i]);
  }
  VectorXd initialState;
  traj_in[0]->getStateForJacobian(initialState);

  WorldSQP *wsqp = 
    new WorldSQP(traj_out.size(), initialState.size());
  wsqp->set_namestring(namestring);
  wsqp->iterative_control_opt(traj_out, control_out, sqp_debug_data, num_iters);

};

void solveSQP(vector<World*>& traj_in, vector<World*>& traj_out, vector<VectorXd>& control_out, const char* namestring, bool planner = true)
{
  vector<vector<World*> > sqp_debug_data;
  solveSQP(traj_in, traj_out, control_out, sqp_debug_data, namestring, planner);
};


void sample_on_sphere(VectorXd& u, const double norm) {
  // sample coordinate wise 
  for (int i = 0; i < u.size(); i++) { 
    u(i) = drand48();
  }
  // place on the unit-ball
  u.normalize();
  // coordinate wise transform to norm-ball 
  for(int i = 0; i < u.size(); i++) { 
    if (drand48() < 0.5) { 
      u(i) *= -norm;
    } else { 
      u(i) *= norm; 
    }
  }
}

void openLoopController(vector<World*> traj_in, vector<VectorXd>& controls_in, vector<World*>& traj_out) {
  World* world = new World(*traj_in[0]);
  for (int i = 0; i < controls_in.size(); i++) {
    traj_out.push_back(new World(*world));
    world->applyRelativeControlJacobian(controls_in[i]);
  }
  traj_out.push_back(new World(*world));
}

void openLoopController(vector<World*> traj_in, vector<vector<Control*> >& controls_in, vector<World*>& traj_out) {
  World* world = new World(*traj_in[0]);
  boost::progress_display progress(controls_in.size());
  for (int i = 0; i < controls_in.size(); i++) {
    traj_out.push_back(new World(*world));
    world->applyRelativeControl(controls_in[i], true);
    ++progress; 
  }
  traj_out.push_back(new World(*world));
}

void closedLoopSQPController(World* start, vector<World*> follow_traj, vector<vector<Control*> >& controls_in, vector<World*>& traj_out) {

  //TODO: FIX NAMESTRING
  //want to minimize actual_i - follow_traj[i]
  int num_sqp_worlds = 10;
  double sqp_init_norm = 1e-1;
  World* start_copy = new World(*start);
  traj_out.push_back(new World(*start_copy));
  for (int i = 0; i < follow_traj.size() - 1; i++) {

    start_copy->applyRelativeControl(controls_in[i], true);

    if (cost_metric(start_copy, follow_traj[i+1]) > SQP_BREAK_THRESHOLD) {
      World* initial_world = new World(*start_copy);
      vector<World*> initialization_worlds;
      initialization_worlds.push_back(new World(*initial_world));
      for (int j = 0; j < num_sqp_worlds - 2; j++) {
        VectorXd du(12);
        sample_on_sphere(du, sqp_init_norm); 
        initial_world->applyRelativeControlJacobian(du);
        initialization_worlds.push_back(new World(*initial_world));
      }
      initialization_worlds.push_back(new World(*follow_traj[i+1]));
      vector<World*> sqpWorlds;
      vector<VectorXd> sqpControls;
      solveSQP(initialization_worlds, sqpWorlds, sqpControls, "sqp");
      vector<World*> openLoopWorlds;
      openLoopController(initialization_worlds, sqpControls, openLoopWorlds);
      
      start_copy = new World(*openLoopWorlds.back());

    }

    traj_out.push_back(new World(*start_copy));

  }
}
void getTrajectoryStatistics(vector<World*>& worlds) {
  // curvature
  // energy
  // twist
  // velocity
  // collisions
  // sensitivity to control?
  // topological? 

  ofstream file;
  file.open("traj_stats.txt"); 
  Thread* lastThread = NULL; 

  for (int i = 0; i < worlds.size(); i++) { 
    vector<ThreadConstrained*> world_threads;
    worlds[i]->getThreads(world_threads); // not copies, so don't mess with it
    //for (int j = 0; j < world_threads.size(); j++) {
    for (int j = 0; j < 1; j++) {
      vector<Thread*> threads;
      world_threads[j]->getThreads(threads);
      Thread* thread = threads.front();

      if (lastThread) {
        VectorXd lastThreadState, currentThreadState;
        lastThread->getState(lastThreadState);
        thread->getState(currentThreadState); 
        file << thread->calculate_energy() << " "; //energy
        file << thread->end_angle() << " "; // twist
        file << (lastThreadState-currentThreadState).norm() << " "; //vel
      }

      lastThread = thread; 

      file << endl; 
    }
  }

  file.close();
}

void getWaypoints(vector<World*>& worlds, vector<World*>& waypoints) { 
  double energy_change_eps = 0.0029; 
  Thread* lastThread = NULL;
  Thread* lastWaypoint = NULL;

  for (int i = 0; i < worlds.size(); i++) { 
    vector<ThreadConstrained*> world_threads;
    worlds[i]->getThreads(world_threads);
    for (int j = 0; j < 1; j++) { 
      vector<Thread*> threads;
      world_threads[j]->getThreads(threads);
      Thread* thread = threads.front(); 
      if (lastThread) {
        if (abs(thread->calculate_energy() - lastThread->calculate_energy()) > energy_change_eps) {
          waypoints.push_back(new World(*worlds[i]));
        }

      }
      lastThread = thread;

    }
  }

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




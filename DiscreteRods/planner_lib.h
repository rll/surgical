#define NUM_ITERS_SQP_PLANNER 1
#define NUM_ITERS_SQP_SMOOTHER 3
#define SQP_BREAK_THRESHOLD 2
#define NOISE_THRESHOLD 0.0


#include "../MotionPlanning/worldSQP.h"
#include "IO/Control.h"
#include <iostream>
#include <sys/time.h>
#include <boost/progress.hpp>


int min(int a, int b) {
  if (a < b) return a;
  else return b; 
}


double distanceBetweenAngles (double a, double b) {
  double dist = fabs(a-b); 
  if (dist > M_PI) {
    dist = 2*M_PI - dist; 
  }
  return dist; 
}

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
void solveSQP(vector<vector<World*> >& traj_in, vector<vector<Control *> >& control_in, vector<vector<World*> >& traj_out, vector<vector<Control *> >& control_out, const char* namestring, bool planner = true)
{
  int num_iters;
  if (planner) {
    num_iters = NUM_ITERS_SQP_PLANNER;
  } else {
    num_iters = NUM_ITERS_SQP_SMOOTHER;
  }

  traj_out.resize(traj_in.size());
  control_out.resize(control_in.size());
  for (int i = 0; i < traj_in.size(); i++) {
    traj_out[i].resize(traj_in[i].size());
    for (int j = 0; j < traj_in[i].size(); j++) { 
      traj_out[i][j] = new World(*traj_in[i][j]);
    }
  }

  for (int i = 0; i < control_in.size(); i++) {
    control_out[i].resize(control_in[i].size());
    for (int j = 0; j < control_in[j].size(); j++) { 
      control_out[i][j] = new Control(*control_in[i][j]);
    }
  }

  VectorXd initialState;
  traj_in[0][0]->getStateForJacobian(initialState);

  WorldSQP *wsqp = 
    new WorldSQP(traj_out[0].size(), initialState.size(), traj_out.size());
  wsqp->set_namestring(namestring);
  wsqp->iterative_control_opt(traj_out, control_out, num_iters);

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
  World* world = new World(*traj_in[0], test_world_manager);
  for (int i = 0; i < controls_in.size(); i++) {
    traj_out.push_back(new World(*world));
    world->applyRelativeControlJacobian(controls_in[i]);
  }
  traj_out.push_back(new World(*world));
}

void openLoopController(vector<World*> traj_in, vector<vector<Control*> >& controls_in, vector<World*>& traj_out) {
  World* world = new World(*traj_in[0], test_world_manager);
  boost::progress_display progress(controls_in.size());
  for (int i = 0; i < controls_in.size(); i++) {
    traj_out.push_back(new World(*world));
    world->applyRelativeControl(controls_in[i], NOISE_THRESHOLD, true);
    ++progress; 
  }
  traj_out.push_back(new World(*world));
}

void openLoopController(World* start, vector<World*> follow_traj, vector<vector<Control*> >& controls_in, vector<World*>& traj_out) { 
  World* world = new World(*start, test_world_manager);
  boost::progress_display progress(controls_in.size());

  for (int i = 0; i < controls_in.size(); i++) {
    traj_out.push_back(new World(*world));
    //cout << cost_metric(world, follow_traj[i]) << endl;
    world->applyRelativeControl(controls_in[i], NOISE_THRESHOLD, true);
    ++progress; 

  }
  traj_out.push_back(new World(*world));
  //cout << cost_metric(world, follow_traj.back()) << endl; 

}

/*void closedLoopSQPController(World* start, vector<World*> follow_traj, vector<vector<Control*> >& controls_in, vector<World*>& traj_out, string sqp_name_string = "clSQP") {

  //TODO: FIX NAMESTRING
  //want to minimize actual_i - follow_traj[i]
  int num_sqp_worlds = 5;
  double sqp_init_norm = 1e-1;
  double sqp_error_to_break = 0;
  World* start_copy = new World(*start);
  traj_out.push_back(new World(*start_copy));
  boost::progress_display progress(controls_in.size());
  for (int i = 0; i < follow_traj.size() - 1; i++) {

    //start_copy->applyRelativeControl(controls_in[i], NOISE_THRESHOLD, true);
    traj_out.push_back(new World(*start_copy));
    
    
    // check if end_effectors closed/open TODO: UNHACK!
    vector<Cursor*> current_cursors;
    start_copy->getObjects<Cursor>(current_cursors);
    vector<Cursor*> goal_cursors;
    follow_traj[i+1]->getObjects<Cursor>(goal_cursors);

    VectorXd du(8*current_cursors.size());
    du.setZero();
    for (int j = 0; j < current_cursors.size(); j++) {
      if (current_cursors[j]->isOpen() != 
          goal_cursors[j]->isOpen()) {
        cout << "Grasping!" << endl; 
        du(8*j + 6) = 1;
      }
    }

    start_copy->applyRelativeControl(du);

    for (int j = 0; j < current_cursors.size(); j++) {
      if (current_cursors[j]->isOpen() != 
          goal_cursors[j]->isOpen()) {
        cout << "wait what?" << endl; 
      }
    }

    cout << "[" << i << ", " << cost_metric(start_copy, follow_traj[i+1]) <<
      "]" << endl; 
    double lastError, nextError;
    if (cost_metric(start_copy, follow_traj[i+1]) > SQP_BREAK_THRESHOLD) {

      //int planInd = min(num_sqp_worlds, follow_traj.size()-i-1);
      int planInd = 1; 
        do { 
        //while (cost_metric(start_copy, follow_traj[i+1]) > SQP_BREAK_THRESHOLD) { 
          World* initial_world = new World(*start_copy);
          vector<World*> initialization_worlds;
          initialization_worlds.push_back(new World(*initial_world));
          for (int j = 0; j < num_sqp_worlds - 2; j++) {
            VectorXd du(12);
            sample_on_sphere(du, sqp_init_norm); 
            initial_world->applyRelativeControlJacobian(du);
            initialization_worlds.push_back(new World(*initial_world));
          }
          lastError = cost_metric(start_copy, follow_traj[i+planInd]);
          initialization_worlds.push_back(new World(*follow_traj[i+planInd]));
          vector<World*> sqpWorlds;
          vector<VectorXd> sqpControls;
          solveSQP(initialization_worlds, sqpWorlds, sqpControls, sqp_name_string.c_str());
          vector<World*> openLoopWorlds;
          openLoopController(initialization_worlds, sqpControls, openLoopWorlds);
          nextError = cost_metric(openLoopWorlds.back(), follow_traj[i+planInd]);
          if (nextError + sqp_error_to_break < lastError) { 
            start_copy = new World(*openLoopWorlds.back());
            for (int i = 0; i < openLoopWorlds.size(); i++) {
              traj_out.push_back(new World(*openLoopWorlds[i]));
            }
          }

        } while (nextError + sqp_error_to_break < lastError); 
          
        //i += planInd;
    }
    ++progress; 

  }
}*/

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
    worlds[i]->getObjects<ThreadConstrained>(world_threads); // not copies, so don't mess with it
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
    worlds[i]->getObjects<ThreadConstrained>(world_threads);
    //worlds[i]->getThreads(world_threads);
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




#include <stdlib.h>

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

#include "IO/Control.h"

#include "EnvObjects/World.h"
#include "EnvObjects/WorldManager.h"

#include "thread_discrete.h"
#include "ThreadConstrained.h"
#include "planner_lib.h"

#include "StateRecorder.h"
#include "StateReader.h"
#include "TrajectoryRecorder.h"
#include "TrajectoryReader.h"

using namespace std;

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

WorldManager* test_world_manager;

VectorXd closedLoopSQPStepper(World* start, World* goal, WorldSQP* solver, double noise_thresh) { 
  if (solver == NULL) assert(false); //initialize your shit
  
  solver->popStart(); //pop the initial start 
  solver->popStart(); //pop the target start

  vector<World*> perturbations;
  perturbations.push_back(start);
  for (int i = 1; i < solver->num_traj(); i++) { 
   World* perturbed_world = new World(*start, test_world_manager);
   VectorXd du(12);
   sample_on_sphere(du, noise_thresh);
   perturbed_world->applyRelativeControlJacobian(du);
   World* input = new World(*perturbed_world);
   perturbations.push_back(input);
   delete perturbed_world;
  }

  solver->pushStart(perturbations); 
  solver->pushGoal(goal);

  for (int i = 0; i < perturbations.size(); i++) {
    //cout << &(*perturbations[i]) << endl; 
    //delete perturbations[i];
  }

  solver->solve();
  return solver->getStartControl();
}

int main (int argc, char * argv[])
{
  test_world_manager = new WorldManager();
  
  if (argc != 8) {
    cout << argc << " is the wrong number of arguments. There should be 8:" << endl;
    cout << "traj_out_filename control_in_filename traj_in_filename start_ind end_ind single_horizon noise_thresh." << endl;
    return 0;
  }
  
  char *traj_out_filename = new char[256];
  char *control_in_filename = new char[256];
  char *traj_in_filename = new char[256];
  sprintf(traj_out_filename, "%s%s", "environmentFiles/", argv[1]);
  sprintf(control_in_filename, "%s%s", "environmentFiles/", argv[2]);  
  sprintf(traj_in_filename, "%s%s", "environmentFiles/", argv[3]);
  
  int start_ind = atoi(argv[4]);
  int end_ind = atoi(argv[5]);
  int single_horizon = atoi(argv[6]);
  double noise_thresh = atof(argv[7]);
  
  vector<vector<Control*> > ctrls;
  TrajectoryReader control_in_reader(control_in_filename);
  if (control_in_reader.readControlsFromFile(ctrls)) {
    cout << "Loading input controls was sucessful. " << ctrls.size() << " controls were loaded." << endl;
  } else {
    cout << "Failed to load input controls from file " << control_in_filename << endl;
    return 0;
  }

  TrajectoryReader traj_in_reader(traj_in_filename);
  vector<World*> traj_in;
  if (traj_in_reader.readWorldsFromFile(traj_in)) {
    cout << "Loading input trajectory was sucessful. " << traj_in.size() << " worlds were loaded." << endl;
  } else {
    cout << "Failed to load input trajectory from file " << traj_in_filename << endl;
    return 0;
  }

  if (start_ind > end_ind) {
    cout << "The index of the start state is higher than the index of the end state." << endl;
    return 0;
  }

  if (start_ind < 0 || end_ind < 0 || start_ind >= traj_in.size() || end_ind >= traj_in.size()) {
    cout << "The index for the input trajectory is out of bounds." << endl;
    return 0;
  }

  World* start = traj_in[start_ind];
  
  vector<int> horizon;
  horizon.push_back(single_horizon);
  //horizon.push_back(0);
  //horizon.push_back(30);
  //horizon.push_back(20);

  vector<vector<World*> > horizon_trajs;
  horizon_trajs.resize(horizon.size());

  for (int h = 0; h < horizon.size(); h++) { 
    cout << "horizon: " << horizon[h] << endl; 

    if (horizon[h] == 0) { 
      vector<World*> openLoopWorlds; 
      openLoopWorlds.push_back(new World(*start));

      World* OLcopy = new World(*start, test_world_manager);
      for (int i = start_ind+1; i < end_ind; i++) {
        cout << "Step " << i << " / " << end_ind << endl;
        //if (interruptEnabled) break; 
        OLcopy->applyRelativeControl(ctrls[i-1], noise_thresh, true);
        openLoopWorlds.push_back(new World(*OLcopy));
      }
      delete OLcopy; 
      horizon_trajs[h] = openLoopWorlds;

    } else { 
      int T = horizon[h];
      vector<World*> init_worlds;
      for (int i = 0; i < T; i++) {
        init_worlds.push_back(traj_in[i]); // solver will make copies
      }
      vector<vector<World*> > sqp_init;
      sqp_init.push_back(init_worlds);
      sqp_init.push_back(init_worlds);
      sqp_init.push_back(init_worlds);
      sqp_init.push_back(init_worlds);

      WorldSQP* solver; 
      solver = new WorldSQP(0,0,0); /// HACK!!
      char namestring[128];
      sprintf(namestring, "clsqp_stepper_h_%d", horizon[h]);
      cout << "namestring = " << namestring << endl; 
      solver->set_namestring(namestring);
      solver->initializeClosedLoopStepper(start, sqp_init);
      solver->solve();

      vector<World*> closedLoopWorlds;
      closedLoopWorlds.push_back(new World(*start));
      VectorXd ctrl_cl_0 = solver->getStartControl();

      World* CLcopy = new World(*start, test_world_manager);
      //start_copy->applyRelativeControlJacobian(ctrl_cl_0);
      //closedLoopWorlds.push_back(new World(*start_copy));

      for (int i = start_ind+1; i < end_ind; i++) {
        cout << "Step " << i << " / " << end_ind << endl;
        //if (interruptEnabled) break; 
        if (i + T > traj_in.size() - 1) T = traj_in.size() - i - 1;
        CLcopy->applyRelativeControl(ctrls[i-1], noise_thresh, true);  
        VectorXd cl_ctrl = closedLoopSQPStepper(CLcopy, traj_in[i+T], solver, noise_thresh);
        CLcopy->applyRelativeControlJacobian(cl_ctrl, noise_thresh);
        closedLoopWorlds.push_back(new World(*CLcopy));
      }
      delete CLcopy;

      horizon_trajs[h] = closedLoopWorlds;
    }
    //if (interruptEnabled) break; 

  }
  //solver->getCurrentStates(closedLoopWorlds);

  TrajectoryRecorder traj_out_recorder(traj_out_filename);
  traj_out_recorder.start();
  traj_out_recorder.writeWorldToFile(start);
  for (int i = 0; i < horizon_trajs[0].size(); i++) {
    traj_out_recorder.writeWorldToFile(horizon_trajs[0][i]);
  }
  traj_out_recorder.stop();
  
  return 1;
}

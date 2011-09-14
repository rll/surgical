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

void chunkSmoother(vector<World*>& traj_in, vector<World*>& traj_out, vector<vector<Control*> >& controls_out, int size_each_chunk, char * namestring) {
	cout << traj_in.size() << " " << size_each_chunk << endl;
	assert((traj_in.size()%size_each_chunk) == 0);
	int num_chunks = traj_in.size() / size_each_chunk;

	vector<vector<World*> > chunks;

	for (int i = 0; i < num_chunks; i++) {
	  vector<World*> individual_chunk;
	  for (int j = 0; j < size_each_chunk; j++) {
	    individual_chunk.push_back(traj_in[i*size_each_chunk + j]);
	  }
	  chunks.push_back(individual_chunk);
	}

	vector<vector<World*> > smooth_chunks;
	vector<vector<VectorXd> > smooth_controls;
	smooth_chunks.resize(chunks.size());
	smooth_controls.resize(chunks.size());

	//#pragma omp parallel for
	for (int i = 0; i < chunks.size(); i++) {
		cout << "On chunk " << i << " of " << chunks.size() << endl; 
	  //smooth each chunk
	  vector<vector<World*> > smooth_chunk;
	  vector<VectorXd> smooth_control;
	  vector<vector<World*> > sqp_init;
	  sqp_init.push_back(chunks[i]);

	  solveSQP(sqp_init, smooth_chunk, smooth_control, namestring, false);
	  VectorXd du(12);
	  du.setZero(); 
	  smooth_chunks[i] = smooth_chunk[0];
	  smooth_controls[i] = smooth_control;
	  smooth_controls[i].push_back(du);
	}

	for (int i = 0; i < smooth_chunks.size(); i++) { 
	  for (int j = 1; j < smooth_chunks[i].size()-1; j++) {
	    traj_out.push_back(smooth_chunks[i][j]);
	    vector<Control*> du;
	    VectorXd full_control = smooth_chunks[i][j]->JacobianControlWrapper(smooth_controls[i][j]);
	    smooth_chunks[i][j]->VectorXdToControl(full_control, du);
	    controls_out.push_back(du);
	  }
	}
	
	for (int i = 0; i < traj_out.size() - 1; i++) {
		vector<Cursor*> pre_cursors;
		traj_out[i]->getObjects<Cursor>(pre_cursors);
		vector<Cursor*> post_cursors;
		traj_out[i+1]->getObjects<Cursor>(post_cursors);
		assert(pre_cursors.size() == post_cursors.size());
		assert(controls_out[i].size() == pre_cursors.size());
		for (int j = 0; j < pre_cursors.size(); j++) {
			if (pre_cursors[j]->isOpen() != post_cursors[j]->isOpen())
				controls_out[i][j]->setButton(UP, true);
		}
	}
	controls_out.pop_back();
}

int main (int argc, char * argv[])
{
  test_world_manager = new WorldManager();
  
  if (argc != 7) {
    cout << argc << " is the wrong number of arguments. There should be 8:" << endl;
    cout << "traj_out_filename control_out_filename traj_in_filename start_ind end_ind size_each_chunk." << endl;
    return 0;
  }
  
  char *traj_out_filename = new char[256];
  char *control_out_filename = new char[256];
  char *traj_in_filename = new char[256];
  sprintf(traj_out_filename, "%s%s", "environmentFiles/", argv[1]);
  sprintf(control_out_filename, "%s%s", "environmentFiles/", argv[2]);  
  sprintf(traj_in_filename, "%s%s", "environmentFiles/", argv[3]);
  
  int start_ind = atoi(argv[4]);
  int end_ind = atoi(argv[5]);
  int size_each_chunk = atoi(argv[6]);

  TrajectoryReader traj_in_reader(traj_in_filename);
  vector<World*> traj_in;
  if (traj_in_reader.readWorldsFromFile(traj_in)) {
    cout << "Loading input trajectory was sucessful. " << traj_in.size() << " worlds were loaded." << endl;
  } else {
    cout << "Failed to load input trajectory from file " << traj_in_filename << endl;
    return 0;
  }

  start_ind = start_ind % traj_in.size();
  end_ind = (traj_in.size()+end_ind) % traj_in.size();
  
  cout << "Using start_ind = " << start_ind << " and end_ind = " << end_ind << endl;
  
  if (start_ind > end_ind) {
    cout << "The index of the start state is higher than the index of the end state." << endl;
    return 0;
  }
  
  vector<World*> traj_in_subset;
  for (int i = start_ind; i <= end_ind; i++) {
  	traj_in_subset.push_back(traj_in[i]);
  }
  
  vector<World*> traj_out;
  vector<vector<Control*> > controls_out;
  char namestring[1024];
  sprintf(namestring, "sqp_smoother_chunk_%s_%s_%s_%s_%s_%s", argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);
  
	chunkSmoother(traj_in_subset, traj_out, controls_out, size_each_chunk, namestring);

  TrajectoryRecorder traj_out_recorder(traj_out_filename);
  traj_out_recorder.start();
  for (int i = 0; i < traj_out.size(); i++) {
    traj_out_recorder.writeWorldToFile(traj_out[i]);
  }
  traj_out_recorder.stop();
  
  TrajectoryRecorder control_out_recorder(control_out_filename);
  control_out_recorder.start();
  for (int i = 0; i < controls_out.size(); i++) {
  	assert(controls_out[i].size() == 2);
    control_out_recorder.writeControlToFile(controls_out[i][0], controls_out[i][1]);
  }
  control_out_recorder.stop();
  
  return 1;
}

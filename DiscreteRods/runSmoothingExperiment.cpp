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

void chunkSmoother(vector<World*>& traj_in, vector<vector<Control*> >& controls_in, vector<World*>& traj_out, vector<vector<Control*> >& controls_out, int size_each_chunk, char *namestring) {
	cout << traj_in.size() << " " << size_each_chunk << endl;
	assert((traj_in.size()%size_each_chunk) == 0);
	int num_chunks = traj_in.size() / size_each_chunk;

  vector<vector<World*> > chunks;
  vector<vector<vector<Control*> > > chunk_ctrls; 

  for (int i = 0; i < num_chunks; i++) {
    vector<World*> individual_chunk;
    vector<vector<Control*> > individual_ctrls; 
    for (int j = 0; j < size_each_chunk; j++) {
      individual_chunk.push_back(traj_in[i*size_each_chunk + j]);
      individual_ctrls.push_back(controls_in[i*size_each_chunk +j]);
    }
    chunks.push_back(individual_chunk);
    chunk_ctrls.push_back(individual_ctrls);
  }

  vector<vector<World*> > smooth_chunks;
  vector<vector<vector<Control *> > > smooth_controls;

  smooth_chunks.resize(chunks.size());
  smooth_controls.resize(chunks.size());

  #pragma omp parallel for 
  for (int i = 0; i < chunks.size(); i++) {
    //smooth each chunk
    char full_namestring[1024];
    sprintf(full_namestring, "%s_%d", namestring, i);
    vector<vector<World*> > smooth_chunk;
    vector<vector<Control*> > smooth_control;
    vector<vector<World*> > sqp_init;
    sqp_init.push_back(chunks[i]);
    chunk_ctrls[i].pop_back();
    solveSQP(sqp_init, chunk_ctrls[i], smooth_chunk, smooth_control, full_namestring, false);
    vector<Control *>  du;
    for (int j = 0; j < 2; j++) {
      du.push_back(new Control(Vector3d::Zero(), Matrix3d::Identity()));
    }
    smooth_control.push_back(du);
    smooth_chunks[i] = smooth_chunk[0];
    smooth_controls[i] = smooth_control;
  }

  for (int i = 0; i < smooth_chunks.size(); i++) { 
    for (int j = 1; j < smooth_chunks[i].size()-1; j++) {
      traj_out.push_back(smooth_chunks[i][j]);
      controls_out.push_back(smooth_controls[i][j]);
    }
  }

	controls_out.pop_back();
}

int main (int argc, char * argv[])
{
  test_world_manager = new WorldManager();
  
  if (argc != 8) {
    cout << argc << " is the wrong number of arguments. There should be 8:" << endl;
    cout << "traj_out_filename control_out_filename traj_in_filename control_in_filename start_ind end_ind size_each_chunk." << endl;
    return 0;
  }
  
  char *traj_out_filename = new char[256];
  char *control_out_filename = new char[256];
  char *traj_in_filename = new char[256];
  char *control_in_filename = new char[256];
  sprintf(traj_out_filename, "%s%s", "environmentFiles/", argv[1]);
  sprintf(control_out_filename, "%s%s", "environmentFiles/", argv[2]);  
  sprintf(traj_in_filename, "%s%s", "environmentFiles/", argv[3]);
  sprintf(control_in_filename, "%s%s", "environmentFiles/", argv[4]);
  
  int start_ind = atoi(argv[5]);
  int end_ind = atoi(argv[6]);
  int size_each_chunk = atoi(argv[7]);

  TrajectoryReader traj_in_reader(traj_in_filename);
  vector<World*> traj_in;
  if (traj_in_reader.readWorldsFromFile(traj_in)) {
    cout << "Loading input trajectory was sucessful. " << traj_in.size() << " worlds were loaded." << endl;
  } else {
    cout << "Failed to load input trajectory from file " << traj_in_filename << endl;
    return 0;
  }

  vector<vector<Control*> > controls_in;
  TrajectoryReader control_traj_reader(control_in_filename); 
  if (control_traj_reader.readControlsFromFile(controls_in)) {
    cout << "Loading input controls was successful. " << controls_in.size() << " controls were loaded." << endl;
  } else {
    cout << "Failed to lead controls from file " << control_in_filename << endl;
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
  vector<vector<Control*> > control_in_subset; 
  for (int i = start_ind; i <= end_ind; i++) {
  	traj_in_subset.push_back(traj_in[i]);
    control_in_subset.push_back(controls_in[i]);
  }
  
  vector<World*> traj_out;
  vector<vector<Control*> > controls_out;
  char namestring[1024];
  sprintf(namestring, "sqp_smoother_chunk_%s_%s_%s_%s_%s_%s_%s", argv[1], argv[2], argv[3], argv[4], argv[5], argv[6], argv[7]);
  
	chunkSmoother(traj_in_subset, control_in_subset, traj_out, controls_out, size_each_chunk, namestring);

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

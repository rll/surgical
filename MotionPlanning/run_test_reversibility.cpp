#include "planner_lib.h"
#include "global_filenames.h"
#include "../DiscreteRods/trajectory_reader.h"
#include "../DiscreteRods/trajectory_recorder.h"
#include <ostream>
#include <boost/timer.hpp>

#define CONTROLS_FILENAME 'reversibility/controls.dat'

void deleteAllThreads(vector<Thread*>& toDelete);
void Copy_Threads(const vector<Thread*>& to_copy, vector<Thread*>& copy);

int main(int argc, char* argv[]) {
  
 if (argc < 3)
 {
    std::cerr << "please provide arguments: start_ind    end_ind   dimension" << std::endl;
 }

 srand(0);
 srand48(0);

  int trajs_start_ind = atoi (argv[1]);
  int trajs_end_ind = atoi (argv[2]);
  int num_links = atoi (argv[3]);
  vector<Thread*> start_threads;
  vector<Thread*> goal_threads;


  char start_threads_filename[256];
  char goal_threads_filename[256];
  sprintf(start_threads_filename, "%s/%s_%d", BASEFOLDER_INITDATA, BASENAME_STARTTHREADS, num_links);
  sprintf(goal_threads_filename, "%s/%s_%d", BASEFOLDER_INITDATA, BASENAME_GOALTHREADS, num_links);

  Trajectory_Reader start_threads_reader(start_threads_filename);
  Trajectory_Reader goal_threads_reader(goal_threads_filename);
  start_threads_reader.read_threads_from_file();
  goal_threads_reader.read_threads_from_file();
  
  //start_threads = start_threads_reader.get_all_threads();
  //goal_threads = goal_threads_reader.get_all_threads();
  

  start_threads_reader.get_all_threads(start_threads);
  goal_threads_reader.get_all_threads(goal_threads);
  

  
  for (int thread_ind = trajs_start_ind; thread_ind <= std::min(trajs_end_ind, (int)(start_threads.size()-1)); thread_ind++)
  {

    char results_filename[256]; 
    sprintf(results_filename, "reversibility/%d_%d_data.txt", num_links, thread_ind); 
    results_file.open(results_filename);

    results_file << "RRT_PLANNER," 
      << cost_metric(RRT_traj.back(), goal_threads[thread_ind])
      << ","
      << RRT_PLANNER_TIME
      << endl;

    results_file.close();
    //interpolate_end_and_linear_recorder.write_threads_to_file();
  } 
}


void deleteAllThreads(vector<Thread*>& toDelete)
{
  /*for (int i=0; i < toDelete.size(); i++)
  {
    delete toDelete[i];
  }*/
  toDelete.resize(0);
}

void Copy_Threads(const vector<Thread*>& to_copy, vector<Thread*>& copy)
{
  copy.resize(to_copy.size());
  for (int i=0; i < to_copy.size(); i++)
  {
    copy[i] = new Thread(*to_copy[i]);
  }
}




  






  






  



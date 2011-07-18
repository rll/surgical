#include "planner_lib.h"
#include "global_filenames.h"
#include "../DiscreteRods/trajectory_reader.h"
#include "../DiscreteRods/trajectory_recorder.h"
#include "../DiscreteRods/controls_reader.h"
#include <ostream>
#include <boost/timer.hpp>

#define CONTROLS_FILENAME "reversibility/controls.dat"

void deleteAllThreads(vector<Thread*>& toDelete);
void Copy_Threads(const vector<Thread*>& to_copy, vector<Thread*>& copy);
void reverseControl(const VectorXd& in_control, VectorXd& out_control); 

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
  
  Controls_Reader controls_reader(CONTROLS_FILENAME);
  vector<VectorXd> all_controls;
  controls_reader.read_controls_from_file();
  controls_reader.get_all_controls(all_controls); 
 
    char results_filename[256]; 
    ofstream results_file; 
    sprintf(results_filename, "reversibility/controls_%d_results.txt", num_links);
    results_file.open(results_filename);

  cout << "Running with threads = " << NUM_CPU_THREADS << endl;
#pragma omp parallel for num_threads(NUM_CPU_THREADS) 
  for (int thread_ind = trajs_start_ind; thread_ind <= std::min(trajs_end_ind, (int)(start_threads.size()-1)); thread_ind++)
  {
    Thread* initial_thread = new Thread(*start_threads[thread_ind]);
    initial_thread->minimize_energy();

    for (int i = 0; i < all_controls.size(); i++) { 

      Thread* copy_thread = new Thread(*initial_thread); 
      VectorXd reverseCtrl(12); 
      reverseControl(all_controls[i], reverseCtrl); 

      applyControl(copy_thread, all_controls[i]);
      results_file << "M " <<  thread_ind << " " << i << " " 
        << cost_metric(copy_thread, initial_thread) << endl; 
      

      applyControl(copy_thread, reverseCtrl);
      //cout << cost_metric(copy_thread, initial_thread)
      //  << endl; 
      results_file << "R " <<  thread_ind << " " << i << " " 
        << cost_metric(copy_thread, initial_thread) << endl;
      /*cout << copy_thread->start_pos().transpose() << endl; 
      cout << initial_thread->start_pos().transpose() << endl;
      cout << copy_thread->end_pos().transpose() << endl; 
      cout << initial_thread->end_pos().transpose() << endl; 
    
      cout << endl << endl; 
      cout << copy_thread->start_rot() << endl; 
      cout << initial_thread->start_rot() << endl; 
      cout << copy_thread->end_rot() << endl; 
      cout << initial_thread->end_rot() << endl; 
      */ 
    }
  } 

    results_file.close();

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

void reverseControl(const VectorXd& in_control, VectorXd& out_control)
{
  Matrix3d start_rot, end_rot, start_rot_rev, end_rot_rev; 
  out_control = in_control;
  out_control(0) = -1 * in_control(0);
  out_control(1) = -1 * in_control(1);
  out_control(2) = -1 * in_control(2);
  rotation_from_euler_angles(start_rot, in_control(3), in_control(4), in_control(5)); 
  start_rot_rev = start_rot.transpose(); 
  euler_angles_from_rotation(start_rot_rev, out_control(3), out_control(4), out_control(5)); 

  rotation_from_euler_angles(start_rot_rev, out_control(3), out_control(4), out_control(5)); 
  //cout << start_rot * start_rot_rev << endl; 

  

  out_control(6) = -1 * in_control(6);
  out_control(7) = -1 * in_control(7);
  out_control(8) = -1 * in_control(8);
  rotation_from_euler_angles(end_rot, in_control(9), in_control(10), in_control(11)); 
  end_rot_rev = end_rot.transpose(); 
  euler_angles_from_rotation(end_rot_rev, out_control(9), out_control(10), out_control(11));

  rotation_from_euler_angles(end_rot_rev, out_control(9), out_control(10), out_control(11));
  //cout << end_rot * end_rot_rev << endl; 


  //std::cout << out_control.transpose() <<  std::endl; 


}



  






  






  



#include "global_filenames.h"
#include "../DiscreteRods/trajectory_reader.h"
#include "../DiscreteRods/trajectory_recorder.h"
#include "../DiscreteRods/controls_reader.h"
#include <ostream>
#include <boost/timer.hpp>
#include "linearization_utils.h"

#define CONTROLS_FILENAME "reversibility/controls.dat"

void deleteAllThreads(vector<Thread*>& toDelete);
void Copy_Threads(const vector<Thread*>& to_copy, vector<Thread*>& copy);
void reverseControl(const VectorXd& in_control, VectorXd& out_control); 
void thread_to_state_with_edges(const Thread* thread, VectorXd& state);
void estimate_transition_matrix_local(Thread* thread, MatrixXd& A);


int main(int argc, char* argv[]) {
  
 if (argc < 3)
 {
    std::cerr << "please provide arguments: start_ind    end_ind   dimension" << std::endl;
 }

 srand(0);
 srand48(0);

  int trajs_start_ind = atoi (argv[1]);
  int trajs_end_ind = atoi (argv[2]);
  //int num_links = atoi (argv[3]);
  
  #pragma omp parallel for num_threads(NUM_CPU_THREADS)
  for (int num_links = 5; num_links <= 19; num_links += 2) {

    vector<Thread*> start_threads;
    vector<Thread*> goal_threads;
    char start_threads_filename[256];
    char goal_threads_filename[256];
    sprintf(start_threads_filename, "reversibility/%s_%d", BASENAME_STARTTHREADS, num_links);
    sprintf(goal_threads_filename, "reversibility/%s_%d", BASENAME_GOALTHREADS, num_links);

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
    sprintf(results_filename, "reversibility/jacobian_%d_results.txt", num_links);
    results_file.open(results_filename);

    //  cout << "Running with threads = " << NUM_CPU_THREADS << endl;
    //#pragma omp parallel for num_threads(NUM_CPU_THREADS) 
    for (int thread_ind = trajs_start_ind; thread_ind <= std::min(trajs_end_ind, (int)(start_threads.size()-1)); thread_ind++)
    {
      if (thread_ind % 10 == 0) {
        cout << "Thread ind: " << thread_ind << endl ; 
      }
      Thread* initial_thread = new Thread(*start_threads[thread_ind]);
      initial_thread->minimize_energy();
      int _size_each_state = 6*initial_thread->num_pieces() - 3 + 1;
      //int _size_each_state = 1 + 3*initial_thread->num_pieces(); 
      int _size_each_control = 12;
      MatrixXd J(_size_each_state, _size_each_control);
      //estimate_transition_matrix_noEdges_withTwist(initial_thread, J, START_AND_END);
      Thread *backup_thread = new Thread(*initial_thread);
      estimate_transition_matrix_local(initial_thread, J);

      VectorXd initial_state(_size_each_state);
      thread_to_state_with_edges(initial_thread, initial_state); 

      for (int i = 0; i < all_controls.size(); i++) { 

        //cout << all_controls[i].transpose() << endl; 
        VectorXd offset(_size_each_state); 
        offset = J * all_controls[i]; // Jacobian estimate
        Thread* copy_thread = new Thread(*backup_thread); 
        applyControl(copy_thread, all_controls[i]);
        VectorXd actual_state(_size_each_state);
        thread_to_state_with_edges(copy_thread, actual_state);

        //cout << offset.transpose() << endl; 
        //cout << (actual_state - initial_state).transpose() << endl; 

        VectorXd actual_offset = actual_state - initial_state; 
        //double error = (offset - actual_offset).norm(); 
        //results_file << thread_ind << " " << i << " " << error << endl;

        //results_file << thread_ind << " " << i << " " << offset.transpose() << actual_offset.transpose() << endl; 

        results_file << offset.transpose() << " " << actual_offset.transpose() << endl; 

        /*
           double similarity = (offset.dot(actual_offset)) / (actual_offset.norm() * offset.norm());
           results_file << thread_ind << " " << i << " " << similarity << endl ;
           */
      }
    } 

    results_file.close();
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


void estimate_transition_matrix_local(Thread* thread, MatrixXd& A)
{
  int num_pieces = thread->num_pieces();
  int num_edges = thread->num_edges();
  vector<ThreadPiece*> thread_backup_pieces;
  thread->save_thread_pieces_and_resize(thread_backup_pieces);

  VectorXd du(A.cols());
  const double eps = 1e-4;
  for(int i = 0; i < A.cols(); i++)
  {
    du.setZero();
    
    du(i) = eps ;
    thread->restore_thread_pieces(thread_backup_pieces);
    applyControl(thread, du, START_AND_END);
    for (int piece_ind=0; piece_ind < num_pieces; piece_ind++)
    {
      A.block(piece_ind*3, i, 3,1) = thread->vertex_at_ind(piece_ind);
    }
    for (int piece_ind=0; piece_ind < num_edges; piece_ind++)
    {
      A.block(3*num_pieces + piece_ind*3, i, 3,1) = thread->edge_at_ind(piece_ind);
    }
    A(6*num_pieces-3, i) = thread->end_angle();

    du(i) = -eps;
    thread->restore_thread_pieces(thread_backup_pieces);
    applyControl(thread, du, START_AND_END);
    for (int piece_ind=0; piece_ind < num_pieces; piece_ind++)
    {

      A.block(piece_ind*3, i, 3,1) -= thread->vertex_at_ind(piece_ind);
    }
    for (int piece_ind=0; piece_ind < num_edges; piece_ind++)
    {
      A.block(3*num_pieces + piece_ind*3, i, 3,1) -= thread->edge_at_ind(piece_ind);
    }

    A(6*num_pieces-3, i) -= thread->end_angle();
  }
  //A /= 2.0*eps; // wtf?
  A /= eps; 
  thread->restore_thread_pieces(thread_backup_pieces);
}



void thread_to_state_with_edges(const Thread* thread, VectorXd& state)
{
  const int num_pieces = thread->num_pieces();
  state.resize(6*num_pieces-3 + 1);
  for (int piece_ind=0; piece_ind < thread->num_pieces(); piece_ind++)
  {
    state.segment(piece_ind*3, 3) = thread->vertex_at_ind(piece_ind);
    if (piece_ind < thread->num_edges()) { 
      state.segment(piece_ind*3 + 3*num_pieces, 3) = thread->edge_at_ind(piece_ind);
    }
  }

  state(6*num_pieces - 3) = thread->end_angle();

  //state(3*num_pieces) = thread->end_angle();
}

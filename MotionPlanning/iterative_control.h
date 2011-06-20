#ifndef _iterative_control_h
#define _iterative_control_h


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/Sparse>
#include <Eigen/QR>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <math.h>
#include "linearization_utils.h"
#include "../DiscreteRods/thread_discrete.h"
#include <fstream>
#include <string.h>
#include "trajectory_follower.h"

#define SQP_BASE_FOLDER "SQP_DATA"
#define FILENAME_ALLTRANS "alltrans.txt"
#define FILENAME_GOALVEC "goalvec.txt"
#define FILENAME_STATEVEC_BASE "newstate"


using namespace Eigen;

static const int _size_each_control = 12;

class Iterative_Control
{
  public:
    Iterative_Control(int num_threads, int num_vertices);
    ~Iterative_Control();

    const int num_threads() const {return _num_threads;};
    const int num_vertices() const {return _num_vertices;};

    void openLoopController(vector<Thread*>& traj_in, vector<VectorXd>& controls_in, vector<Thread*>& traj_out) {
      Thread* thread = new Thread(*traj_in[0]);
      for (int i = 0; i < controls_in.size(); i++) {
        traj_out.push_back(new Thread(*thread));
        applyControl(thread, controls_in[i]);
      }
      traj_out.push_back(new Thread(*thread));
    };


    void closedLoopController(vector<Thread*>& traj_in, vector<vector<VectorXd> >& controls_in, vector<Thread*>& traj_out)
    {
      // copy input trajectory 
      vector<Thread*> traj_in_copy;
      traj_in_copy.resize(traj_in.size()-1); 
      for (int i = 1; i < traj_in.size(); i++) {
        traj_in_copy[i-1] = new Thread(*traj_in[i]);
      }

      Thread* start_copy = new Thread(*traj_in[0]);

      // follow using trajectory follower
      Trajectory_Follower *follower = 
        new Trajectory_Follower(traj_in_copy, controls_in, start_copy);
      follower->control_to_finish(); 

      //put states reached in traj_out 
      follower->getReachedStates(traj_out); 
    };

    void resize_controller(int num_threads, int num_vertices);

    bool iterative_control_opt(vector<Thread*>& trajectory, vector<VectorXd>& controls, int num_opts = 5);
    bool iterative_control_opt(vector<Thread*>& trajectory, vector<VectorXd>& controls, vector<Thread*>& sqp_debug_data, int num_opts = 5);
    void AnswerFile_To_Traj(const char* filename, vector<Thread*>& trajectory, vector<VectorXd>& control);
    void AllFiles_To_Traj(int num_iters, vector< vector<Thread*> >& trajectory, vector< vector<VectorXd> >& control);

    void set_namestring(const char* str)
    {
      strcpy(_namestring, str);
    };
    

  private:
    void init_all_trans(); /* adds the diagonal weighting terms to _all_trans */
    void add_transitions_alltrans(vector<Thread*>& trajectory);

    DynamicSparseMatrix<double> _all_trans;
    int _num_threads;
    int _size_each_state;
    int _num_vertices;
    int _cols_all_unknown_states;
    Thread* _lastopt_startThread;
    Thread* _lastopt_goalThread;

    char _namestring[256];

};


void thread_to_state(const Thread* thread, VectorXd& state);
void weight_state(VectorXd& state);
void Matrix_To_File(SparseMatrix<double> mat, const char* filename);
void File_To_Vector(const char* filename, VectorXd& vec);
void Vector_To_File(VectorXd& vec, const char* filename);

#endif

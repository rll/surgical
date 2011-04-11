#ifndef _iterative_control_h
#define _iterative_control_h


#define EIGEN_UMFPACK_SUPPORT
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

#define FILENAME_ALLTRANS "alltrans.txt"
#define FILENAME_GOALVEC "goalvec.txt"
#define FILENAME_STATEVEC "newstate.txt"


using namespace Eigen;

static const int _size_each_control = 12;

class Iterative_Control
{
  public:
    Iterative_Control(int num_threads, int num_vertices);
    ~Iterative_Control();

    const int num_threads() const {return _num_threads;};
    const int num_vertices() const {return _num_vertices;};

    void resize_controller(int num_threads, int num_vertices);

    bool iterative_control_opt(vector<Thread*>& trajectory, vector<VectorXd>& controls, int num_opts = 50);
    

  private:
    void init_all_trans(); /* adds the diagonal weighting terms to _all_trans */
    void add_transitions_alltrans(vector<Thread*>& trajectory);

    DynamicSparseMatrix<double> _all_trans;
    int _num_threads;
    int _size_each_state;
    int _num_vertices;
    int _cols_all_unknown_states;


};


void thread_to_state(const Thread* thread, VectorXd& state);
void weight_state(VectorXd& state);
void Matrix_To_File(SparseMatrix<double> mat, const char* filename);
void File_To_Vector(const char* filename, VectorXd& vec);
void Vector_To_File(VectorXd& vec, const char* filename);

#endif

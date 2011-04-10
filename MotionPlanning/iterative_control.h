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

    MatrixXd _all_trans;
    int _num_threads;
    int _size_each_state;
    int _num_vertices;
    int _num_edges;
    int _cols_all_unknown_states;


};

#endif

#ifndef _world_sqp_h
#define _world_sqp_h


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/Sparse>
#include <Eigen/QR>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <math.h>
#include "../DiscreteRods/EnvObjects/World.h"
#include <fstream>
#include <string.h>
#include <float.h>

#define SQP_BASE_FOLDER "../MotionPlanning/SQP_DATA"
#define FILENAME_ALLTRANS "alltrans.txt"
#define FILENAME_GOALVEC "goalvec.txt"
#define FILENAME_STATEVEC_BASE "newstate"

using namespace Eigen;

static const int _size_each_control = 12;

class WorldSQP
{
  public:
    WorldSQP(int num_worlds, int size_each_state);
    ~WorldSQP();

    const int num_worlds() const {return _num_worlds;};

    void resize_controller(int num_worlds, int size_each_state);

    bool iterative_control_opt(vector<World*>& trajectory, vector<VectorXd>& controls, int num_opts = 5);
    bool iterative_control_opt(vector<World*>& trajectory, vector<VectorXd>& controls, vector<vector<World*> >& sqp_debug_data, int num_opts = 5, bool return_best_opt = true, double threshold = 2);

    void set_namestring(const char* str)
    {
      strcpy(_namestring, str);
    };


    /*
     * Open loop controller applies controls in controls_in at every time step
     * Assumes start thread is traj_in[0]
     */
    void openLoopController(vector<World*>& traj_in, vector<VectorXd>& controls_in, vector<World*>& traj_out) {
      World* world = new World(*traj_in[0]);
      for (int i = 0; i < controls_in.size(); i++) {
        traj_out.push_back(new World(*world));
        world->applyRelativeControlJacobian(controls_in[i]);
      }
      traj_out.push_back(new World(*world));
    }

    double l2PointsDifference(World* a, World* b) { 
      VectorXd state_a, state_b;
      a->getStateForJacobian(state_a);
      b->getStateForJacobian(state_b);
      return l2PointsDifference(state_a, state_b); 
    }

    double l2PointsDifference(VectorXd a, VectorXd b) { 
      return (a - b).norm(); 
    }

    void world_to_state(World* world, VectorXd& state);

  private:
    void init_all_trans();
    void add_transitions_alltrans(vector<World*>& trajectory);
    DynamicSparseMatrix<double> _all_trans;
    int _num_worlds;
    int _size_each_state;
    int _cols_all_unknown_states;
    char _namestring[256];

};

void Matrix_To_File(SparseMatrix<double> mat, const char* filename);
void File_To_Vector(const char* filename, VectorXd& vec);
void Vector_To_File(VectorXd& vec, const char* filename);

#endif

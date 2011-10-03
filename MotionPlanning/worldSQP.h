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
#include "../utils/clock2.h"
#include "../DiscreteRods/EnvObjects/WorldManager.h"
#include "../DiscreteRods/IO/Control.h"

#define SQP_BASE_FOLDER "../MotionPlanning/SQP_DATA"
#define FILENAME_ALLTRANS "alltrans.txt"
#define FILENAME_GOALVEC "goalvec.txt"
#define FILENAME_STATEVEC_BASE "newstate"
#define FILENAME_INIT_CTRLS "controlvec.txt"

using namespace Eigen;

static const int _size_each_control = 12;

class WorldSQP
{
  public:
    WorldSQP(int num_worlds, int size_each_state, int num_traj);
    ~WorldSQP();

    const int num_worlds() const {return _num_worlds;};
    const int num_traj() const {return _num_traj;};

    void resize_controller(int num_worlds, int size_each_state, int num_traj);

    bool iterative_control_opt(vector<vector<World*> >& trajectory, vector<vector<Control *> >& controls, int num_opts = 5, bool return_best_opt = true, double threshold = 2);

    void set_namestring(const char* str)
    {
      strcpy(_namestring, str);
    };

    void initializeClosedLoopStepper(World* start, vector<vector<World*> >& target);
    void pushStart(vector<World*>& state);
    void pushGoal(vector<World*>& state);
    void pushGoal(World* state);
    void pushStart(World* state);
    void popGoal();
    void popStart(); // poptarts anyone?
    void getCurrentStates(vector<vector<World*> >& colocation_states) {
      colocation_states.resize(current_states.size());
      for (int i = 0; i < current_states.size(); i++) {
        colocation_states[i].resize(current_states[i].size());
        for (int j = 0; j < current_states[i].size(); j++) { 
          colocation_states[i][j] = new World(*current_states[i][j]);
        }
      }
    }

    VectorXd getStartControl();
    void solve();

    /*
     * Open loop controller applies controls in controls_in at every time step
     * Assumes start thread is traj_in[0]
     */
    /*void openLoopController(vector<World*>& traj_in, vector<VectorXd>& controls_in, vector<World*>& traj_out) {
      World* world = new World(*traj_in[0], _wsqp_manager);
      for (int i = 0; i < controls_in.size(); i++) {
        traj_out.push_back(new World(*world));
        world->applyRelativeControlJacobian(controls_in[i]);
      }
      traj_out.push_back(new World(*world));
      delete world; 
    }*/
    
    void openLoopController(vector<World*>& traj_in, vector<vector<Control*> >& controls_in, vector<World*>& traj_out) {
      World* world = new World(*traj_in[0], _wsqp_manager);
      for (int i = 0; i < controls_in.size(); i++) {
        traj_out.push_back(new World(*world));
        world->applyRelativeControl(controls_in[i], 0.0, true);
      }
      traj_out.push_back(new World(*world));
      delete world; 

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
    void compute_difference_block(DynamicSparseMatrix<double>& m);
    void compute_all_jacobians();
    //void computeJacobian(int traj_ind, int world_ind);
    //void add_transitions_alltrans(vector<World*>& trajectory);
    DynamicSparseMatrix<double> _all_trans;
    int _num_worlds;
    int _size_each_state;
    int _cols_all_unknown_states;
    int _num_traj;
    vector<vector<World*> >   current_states;
    vector<vector<Control*> > current_controls;
    vector<vector<MatrixXd> > current_jacobians;
    WorldManager* _wsqp_manager;

    char _namestring[256];

};

void Matrix_To_File(SparseMatrix<double> mat, const char* filename);
void File_To_Vector(const char* filename, VectorXd& vec);
void Vector_To_File(VectorXd& vec, const char* filename);
void block(DynamicSparseMatrix<double>& c, int s_row, int s_col, DynamicSparseMatrix<double>& data);
void block(DynamicSparseMatrix<double>& c, int s_row, int s_col, MatrixXd& data);
void computeJacobian(World* w, MatrixXd* J);
void setWorldFromState(World* w, VectorXd* s);
#endif

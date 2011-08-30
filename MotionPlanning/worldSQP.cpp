#include "worldSQP.h"
#include <boost/progress.hpp>

#define MATLAB_INSTALL "matlab"

WorldSQP::WorldSQP(int num_worlds, int size_each_state)
{
  _num_worlds = 0;
  _size_each_state = 0; 
  resize_controller(num_worlds, size_each_state);
  strcpy(_namestring, "");
}

WorldSQP::~WorldSQP()
{

}

void WorldSQP::resize_controller(int num_worlds, int size_each_state)
{
  if (num_worlds == _num_worlds && size_each_state == _size_each_state)
    return;

  _num_worlds = num_worlds;
  _size_each_state = size_each_state;
  _cols_all_unknown_states = (num_worlds-2)*_size_each_state;
  _all_trans.resize(_size_each_state*(_num_worlds-1), (_num_worlds-2)*_size_each_state + (_num_worlds-1)*_size_each_control);
  _all_trans.setZero();

  init_all_trans();
}


bool WorldSQP::iterative_control_opt(vector<World*>& trajectory, vector<VectorXd>& controls, int num_opts) { 
  vector<vector<World*> > sqp_debug_data;
  return iterative_control_opt(trajectory, controls, sqp_debug_data, num_opts);
}



bool WorldSQP::iterative_control_opt(vector<World*>& trajectory, vector<VectorXd>& controls, vector<vector<World*> >& sqp_debug_data, int num_opts, bool return_best_opt, double threshold) 
{

  assert(trajectory.size() == _num_worlds); // memory leak if false


  //vector to contain the new states
  VectorXd new_states((_num_worlds-2)*_size_each_state + (_num_worlds-1)*_size_each_control);

  //setup filenames
  char filename_goalvec[256];
  sprintf(filename_goalvec, "%s/%s_%s", SQP_BASE_FOLDER, _namestring, FILENAME_GOALVEC);
  char filename_alltrans[256];
  sprintf(filename_alltrans, "%s/%s_%s", SQP_BASE_FOLDER, _namestring, FILENAME_ALLTRANS);


  // prepare variables for iteration 
  sqp_debug_data.resize(0); //TODO: sqp_debug_data not implemented! 
  vector<World*> best_trajectory;
  vector<VectorXd> best_controls; 
  double best_score = DBL_MAX;

  // iterate!
  for (int opt_iter=0; opt_iter < num_opts; opt_iter++)
  {
    /* Memory leak, fix later. Needed to get jacobian computation done in parallel */ 
    vector<World*> trajectory_local;
    trajectory_local.resize(trajectory.size());
    #pragma omp parallel for
    for (int i = 0; i < trajectory.size(); i++) { 
      trajectory_local[i] = new World(*trajectory[i]);
    }
    trajectory = trajectory_local; 
  
    //compute Jacobians
    add_transitions_alltrans(trajectory);

    //write out all Jacobians to file
    SparseMatrix<double> all_trans_sparse(_all_trans);
    Matrix_To_File(all_trans_sparse, filename_alltrans);

    //write out all states to file 
    VectorXd goal_vector(_size_each_state*_num_worlds);
    VectorXd state_for_file(_size_each_state);
    for (int i=0; i < trajectory.size(); i++)
    {
      world_to_state(trajectory[i], state_for_file);
      goal_vector.segment(i*_size_each_state, _size_each_state) = state_for_file;
    }
    Vector_To_File(goal_vector, filename_goalvec);
   
    //prepare to execute solver (MATLAB)
    char filename_statevec_thisiter[256];
    sprintf(filename_statevec_thisiter, "%s/%s_%s%d.txt", SQP_BASE_FOLDER, _namestring, FILENAME_STATEVEC_BASE, opt_iter);
    char matlab_command[1024];
    sprintf(matlab_command, "%s -nodisplay -nodesktop -nojvm -r \"solve_sparse(%d, %d, \'%s\', %d, %d, \'%s\', \'%s\', %d, %d, %d)\"", MATLAB_INSTALL, _all_trans.rows(), _all_trans.cols(), filename_alltrans, goal_vector.rows(), goal_vector.cols(), filename_goalvec, filename_statevec_thisiter, _num_worlds, _size_each_state, _size_each_control);
    std::cout << "command: " << matlab_command << std::endl;
    
    //run solver (MATLAB)
    int return_value = system(matlab_command);
    
    //Read solver output
    File_To_Vector(filename_statevec_thisiter, new_states);

    //copy out control
    controls.resize(_num_worlds-1);
    vector<vector<VectorXd> > control_vector;
    for (int i=0; i < _num_worlds-1; i++)
    {
      controls[i] = new_states.segment(_size_each_state*(_num_worlds-2) + i*_size_each_control, _size_each_control);
    }

    //compute OLC Trajectory using controls 
    vector<World*> trajectory_copy;  
    trajectory_copy.push_back(new World(*trajectory[0]));
    vector<World*> OLTrajectory;
    openLoopController(trajectory_copy, controls, OLTrajectory);

    //evalulate open loop trajectory score 
    double sqp_olc_score = l2PointsDifference(OLTrajectory.back(), trajectory.back());
    cout << "SQP OLC score = " << sqp_olc_score << endl;
    
    //set trajectory = OLTrajectory for next iteration 
    OLTrajectory[OLTrajectory.size()-1] = trajectory[trajectory.size()-1];
    trajectory = OLTrajectory;

    //if (return_best_opt && sqp_olc_score < threshold) return true; 
    if (return_best_opt && sqp_olc_score < best_score) {
      best_score = sqp_olc_score;
      // cleanup
      if (best_trajectory.size() > 0) { 
        for (int i = 0; i < best_trajectory.size(); i++) {
          delete best_trajectory[i];
        }
      }
      best_trajectory.resize(trajectory.size());
      for (int i = 0; i < trajectory.size(); i++) { 
          best_trajectory[i] = new World(*trajectory[i]);
      }
      best_controls.resize(controls.size());
      for (int i = 0; i < controls.size(); i++) {
        best_controls[i] = controls[i];
      }
    }
  }

  if (return_best_opt) { 
    trajectory = best_trajectory;
    controls = best_controls;
  }

  return true;
}



void WorldSQP::init_all_trans()
{
  for (int i=0; i < _num_worlds-1; i++)
  {
    int num_rows_start = i*_size_each_state;
    int num_cols_start = _cols_all_unknown_states+i*_size_each_control;
    for (int r=0; r < _size_each_state; r++)
    {
      if (i != 0)
      {
        _all_trans.coeffRef(num_rows_start+r,num_rows_start+r -_size_each_state) = 1;
      }
      if (i != _num_worlds-2)
      {
        _all_trans.coeffRef(num_rows_start+r,num_rows_start+r) = -1;
      }
    }
  }
}

void WorldSQP::add_transitions_alltrans(vector<World*>& trajectory)
{
 
  cout << "Computing Jacobians. " << endl;
  boost::progress_display progress(trajectory.size()-1);
  #pragma omp parallel for num_threads(NUM_CPU_THREADS) 
  for (int i=0; i < trajectory.size()-1; i++)
  {
    MatrixXd trans;
    trajectory[i]->computeJacobian(trans);

    int num_rows_start = i*_size_each_state;
    int num_cols_start = _cols_all_unknown_states+i*_size_each_control;
    for (int r=0; r < _size_each_state; r++)
    {
      for (int c=0; c < _size_each_control; c++)
      {
        _all_trans.coeffRef(num_rows_start+r, num_cols_start+c) = trans(r,c);
      }
    }
    ++progress; 
  }
  cout << "All Jacobians Computed" << endl;
}

void WorldSQP::world_to_state(World* world, VectorXd& state)
{
  world->getStateForJacobian(state); 
}

void Matrix_To_File(SparseMatrix<double> mat, const char* filename)
{
  ofstream toFile(filename);
  toFile.precision(10);
  for (int k=0; k < mat.outerSize(); ++k) {
    for (SparseMatrix<double>::InnerIterator it(mat,k); it; ++it)
    {
      toFile << (it.row()+1) << "\t" << (it.col()+1) << "\t" << it.value() <<"\n";
    }
  }
  toFile.close();
}

void File_To_Vector(const char* filename, VectorXd& vec)
{
  ifstream fromFile(filename);
  char temp_holder[256];
  for (int i=0; i < vec.rows(); i++)
  {
    fromFile >> temp_holder;
    vec(i) = atof(temp_holder);
  }
}

void Vector_To_File(VectorXd& vec, const char* filename)
{
  ofstream toFile(filename);
  toFile << vec << std::endl;
  toFile.close();
}


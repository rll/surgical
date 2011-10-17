#include "iterative_control.h"
#include <boost/progress.hpp>

#define MATLAB_INSTALL "matlab"



Iterative_Control::Iterative_Control(int num_threads, int num_vertices)
{
  _num_threads = 0;
  _num_vertices = 0;
  resize_controller(num_threads, num_vertices);
  strcpy(_namestring, "");
}

Iterative_Control::~Iterative_Control()
{

}

void Iterative_Control::resize_controller(int num_threads, int num_vertices)
{
  if (num_threads == _num_threads && num_vertices == _num_vertices)
    return;

  _num_threads = num_threads;
  _num_vertices = num_vertices;
  _size_each_state = (-3 + 6*num_vertices) + 2;
  //_size_each_state = 3*num_vertices + 2; 
  _cols_all_unknown_states = (num_threads-2)*_size_each_state;
  _all_trans.resize(_size_each_state*(_num_threads-1), (_num_threads-2)*_size_each_state + (_num_threads-1)*_size_each_control);
  _all_trans.setZero();

  init_all_trans();
}


bool Iterative_Control::iterative_control_opt(vector<Thread*>& trajectory, vector<VectorXd>& controls, int num_opts) { 
  vector<vector<Thread*> > sqp_debug_data;
  return iterative_control_opt(trajectory, controls, sqp_debug_data, num_opts);
}



bool Iterative_Control::iterative_control_opt(vector<Thread*>& trajectory, vector<VectorXd>& controls, vector<vector<Thread*> >& sqp_debug_data, int num_opts, bool return_best_opt, double threshold) 
{
  if (trajectory.size() != _num_threads && trajectory.front()->num_pieces() != _num_vertices)
    return false;

  //make a copy of the start and goal threads
  _lastopt_startThread = new Thread(*trajectory.front());
  _lastopt_goalThread = new Thread(*trajectory.back());

  //vector to contain the new states
  VectorXd new_states((_num_threads-2)*_size_each_state + (_num_threads-1)*_size_each_control);

  char filename_goalvec[256];
  sprintf(filename_goalvec, "%s/%s_%s", SQP_BASE_FOLDER, _namestring, FILENAME_GOALVEC);

  char filename_alltrans[256];
  sprintf(filename_alltrans, "%s/%s_%s", SQP_BASE_FOLDER, _namestring, FILENAME_ALLTRANS);

  sqp_debug_data.resize(0); 

  vector<Thread*> best_trajectory;
  vector<VectorXd> best_controls; 
  double best_score = DBL_MAX;

  for (int opt_iter=0; opt_iter < num_opts; opt_iter++)
  {
    /* Memory leak, fix later. Needed to get jacobian computation done in parallel */ 
    vector<Thread*> trajectory_local;
    trajectory_local.resize(trajectory.size());
    #pragma omp parallel for
    for (int i = 0; i < trajectory.size(); i++) { 
      trajectory_local[i] = new Thread(*trajectory[i]);
    }
    trajectory = trajectory_local; 

    add_transitions_alltrans(trajectory);
    SparseMatrix<double> all_trans_sparse(_all_trans);
    Matrix_To_File(all_trans_sparse, filename_alltrans);
    //write all states out
    VectorXd goal_vector(_size_each_state*_num_threads);
    VectorXd state_for_file(_size_each_state);
    for (int i=0; i < trajectory.size(); i++)
    {
      thread_to_state(trajectory[i], state_for_file);
      goal_vector.segment(i*_size_each_state, _size_each_state) = state_for_file;
    }
    Vector_To_File(goal_vector, filename_goalvec);
    //LU<MatrixXd> lu_factorization((MatrixXd(all_trans_sparse.transpose())*all_trans_sparse));
    //VectorXd b = DynamicSparseMatrix<double>(all_trans_sparse.transpose())*goal_vector;
    //new_states = b;
    //lu_factorization.solve(b, &new_states);
    //lu_factorization.solveInPlace(new_states);
    

    //VectorXd c(goal_vector.rows());
    //(_all_trans*_all_trans.transpose()).svd().solve(goal_vector, &c);
 
    //new_states = _all_trans.transpose()*c;

    //std::cout << "error: " << (_all_trans*new_states - goal_vector).norm() << std::endl;

    vector<Vector3d> points(_num_vertices);
    vector<double> angles(_num_vertices);

    char filename_statevec_thisiter[256];
    sprintf(filename_statevec_thisiter, "%s/%s_%s%d.txt", SQP_BASE_FOLDER, _namestring, FILENAME_STATEVEC_BASE, opt_iter);

    char matlab_command[1024];
    sprintf(matlab_command, "%s -nodisplay -nodesktop -nojvm -r \"solve_sparse(%d, %d, \'%s\', %d, %d, \'%s\', \'%s\', %d, %d, %d)\"", MATLAB_INSTALL, _all_trans.rows(), _all_trans.cols(), filename_alltrans, goal_vector.rows(), goal_vector.cols(), filename_goalvec, filename_statevec_thisiter, _num_threads, _size_each_state, _size_each_control);
    std::cout << "command: " << matlab_command << std::endl;

    int return_value = system(matlab_command);
    File_To_Vector(filename_statevec_thisiter, new_states);
    

    cout << "NOT Minimizing Threads returned from MATLAB" << endl; 
    boost::progress_display progress(trajectory.size()-2);
    vector<Thread*> sqp_debug_data_iter;
    sqp_debug_data_iter.resize(trajectory.size());
    sqp_debug_data_iter[0] = new Thread(*trajectory[0]);
    sqp_debug_data_iter[trajectory.size()-1] = new Thread(*trajectory[trajectory.size()-1]);
    #pragma omp parallel for num_threads(NUM_CPU_THREADS)
    for (int i=1; i < trajectory.size()-1; i++)
    {
      VectorXd to_copy = new_states.segment(_size_each_state*(i-1), _size_each_state);
      trajectory[i]->copy_data_from_vector(to_copy);
      sqp_debug_data_iter[i] = new Thread(*trajectory[i]); 
      trajectory[i]->unviolate_total_length_constraint();
      trajectory[i]->project_length_constraint();
      //trajectory[i]->minimize_energy(150000000);
      ++progress;
    }

    sqp_debug_data.push_back(sqp_debug_data_iter);

    //copy out control
    controls.resize(_num_threads-1);
    vector<vector<VectorXd> > control_vector;
    for (int i=0; i < _num_threads-1; i++)
    {
      controls[i] = new_states.segment(_size_each_state*(_num_threads-2) + i*_size_each_control, _size_each_control);
    }

    vector<Thread*> trajectory_copy;  
    trajectory_copy.push_back(new Thread(*trajectory[0]));
    vector<Thread*> OLTrajectory;
    openLoopController(trajectory_copy, controls, OLTrajectory);
    double sqp_olc_score = util_planner.l2PointsDifference(OLTrajectory.back(), trajectory.back());
    cout << "SQP OLC score = " << sqp_olc_score << endl;
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
          best_trajectory[i] = new Thread(*trajectory[i]);
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
    //for (int i = 10; i < controls.size(); i++) {
    //  controls[i].setZero();
    //}
  }

  return true;
}



void Iterative_Control::init_all_trans()
{
  //init the weight matrix for states
  MatrixXd state_weight_mat = MatrixXd::Identity(_size_each_state, _size_each_state);
  for (int i=0; i < 3*_num_vertices; i++)
  {
    state_weight_mat(i,i) = WEIGHT_VERTICES;
  } 
  state_weight_mat(3*_num_vertices, 3*_num_vertices) = WEIGHT_ANGLE;


  for (int i=0; i < _num_threads-1; i++)
  {
    int num_rows_start = i*_size_each_state;
    int num_cols_start = _cols_all_unknown_states+i*_size_each_control;
    for (int r=0; r < _size_each_state; r++)
    {
      if (i != 0)
      {
        _all_trans.coeffRef(num_rows_start+r,num_rows_start+r -_size_each_state) = state_weight_mat(r,r);
      }
      if (i != _num_threads-2)
      {
        _all_trans.coeffRef(num_rows_start+r,num_rows_start+r) = -state_weight_mat(r,r);
      }
    }
  }
}

void Iterative_Control::add_transitions_alltrans(vector<Thread*>& trajectory)
{
 
  cout << "Computing Jacobians. " << endl;
  boost::progress_display progress(trajectory.size()-1);
  #pragma omp parallel for num_threads(NUM_CPU_THREADS) 
  for (int i=0; i < trajectory.size()-1; i++)
  {
    //estimate_transition_matrix_noEdges_withTwist(trajectory[i], trans, START_AND_END);
    MatrixXd trans(_size_each_state, _size_each_control);
    estimate_transition_matrix_withTwist(trajectory[i], trans, START_AND_END);

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


void Iterative_Control::AnswerFile_To_Traj(const char* filename, vector<Thread*>& trajectory, vector<VectorXd>& control)
{
  VectorXd new_states((_num_threads-2)*_size_each_state + (_num_threads-1)*_size_each_control);
  File_To_Vector(filename, new_states);
  trajectory.resize(0);
  trajectory.push_back(new Thread(*_lastopt_startThread));
  for (int i=1; i < _num_threads-1; i++)
  {
    VectorXd to_copy = new_states.segment(_size_each_state*(i-1), _size_each_state);
    trajectory.push_back(new Thread(*trajectory.back()));
    trajectory[i]->copy_data_from_vector(to_copy);
    trajectory[i]->unviolate_total_length_constraint();
    trajectory[i]->project_length_constraint();
    trajectory[i]->minimize_energy();
  }
  trajectory.push_back(new Thread(*_lastopt_goalThread));

  //copy out control
  control.resize(_num_threads-1);
  for (int i=0; i < _num_threads-1; i++)
  {
    control[i] = new_states.segment(_size_each_state*(_num_threads-2) + i*_size_each_control, _size_each_control);
  }
}

void Iterative_Control::AllFiles_To_Traj(int num_iters, vector< vector<Thread*> >& trajectory, vector< vector<VectorXd> >& control)
{
  trajectory.resize(num_iters);
  control.resize(num_iters);
  for (int opt_iter=0; opt_iter < num_iters; opt_iter++)
  {
    char filename_statevec_thisiter[256];
    sprintf(filename_statevec_thisiter, "%s/%s%d.txt", SQP_BASE_FOLDER, FILENAME_STATEVEC_BASE, opt_iter);
    AnswerFile_To_Traj(filename_statevec_thisiter, trajectory[opt_iter], control[opt_iter]);
  }
}

void Iterative_Control::thread_to_state(const Thread* thread, VectorXd& state)
{
  const int num_pieces = thread->num_pieces();
  state.resize(_size_each_state);
  for (int piece_ind=0; piece_ind < thread->num_pieces(); piece_ind++)
  {
    state.segment(piece_ind*3, 3) = thread->vertex_at_ind(piece_ind);
    if (piece_ind < thread->num_edges()) { 
      state.segment(piece_ind*3 + 3*num_pieces, 3) = thread->edge_at_ind(piece_ind);
    }
  }
  state(6*num_pieces - 3) = thread->end_angle();
  state(6*num_pieces - 2) = ((Thread*)thread)->calculate_energy();
  //state(3*num_pieces) = thread->end_angle();
  //state(3*num_pieces+1) = ((Thread*)thread)->calculate_energy();
}

/*
void thread_to_state(const Thread* thread, VectorXd& state)
{
  const int num_pieces = thread->num_pieces();
  state.resize(3*num_pieces+1);
  for (int piece_ind=0; piece_ind < thread->num_pieces(); piece_ind++)
  {
    state.segment(piece_ind*3, 3) = thread->vertex_at_ind(piece_ind);
  }
  state(3*num_pieces) = thread->end_angle();
}
*/
void weight_state(VectorXd& state)
{
  const int num_pieces = (state.rows()-1)/3;
  for (int piece_ind=0; piece_ind < num_pieces; piece_ind++)
  {
    state.segment(piece_ind*3, 3) *= WEIGHT_VERTICES;
  }
  state(3*num_pieces) *= WEIGHT_ANGLE;
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


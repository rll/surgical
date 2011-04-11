#include "iterative_control.h"


Iterative_Control::Iterative_Control(int num_threads, int num_vertices)
{
  _num_threads = 0;
  _num_vertices = 0;
  resize_controller(num_threads, num_vertices);

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
  _size_each_state = (3*num_vertices) + 1;
  _cols_all_unknown_states = (num_threads-2)*_size_each_state;
  _all_trans.resize(_size_each_state*(_num_threads-1), (_num_threads-2)*_size_each_state + (_num_threads-1)*_size_each_control);
  _all_trans.setZero();

  init_all_trans();
}




bool Iterative_Control::iterative_control_opt(vector<Thread*>& trajectory, vector<VectorXd>& controls, int num_opts)
{
  if (trajectory.size() != _num_threads && trajectory.front()->num_pieces() != _num_vertices)
    return false;

  //setup goal vector
  /*
  VectorXd goal_vector( (_num_threads-1)*_size_each_state);
  goal_vector.setZero();
  VectorXd state_for_ends(_size_each_state);
  thread_to_state(trajectory.front(), state_for_ends);
  weight_state(state_for_ends);
  goal_vector.segment(0, _size_each_state) = -1 * state_for_ends;
  thread_to_state(trajectory.back(), state_for_ends);
  weight_state(state_for_ends);
  goal_vector.segment((_num_threads-2)*_size_each_state, _size_each_state) = state_for_ends;
  */
  SparseMatrix<double> goal_vector( (_num_threads-1)*_size_each_state, 1);
  VectorXd state_for_ends(_size_each_state);
  thread_to_state(trajectory.front(), state_for_ends);
  weight_state(state_for_ends);
  goal_vector.startFill(2*_size_each_state);
  for (int i=0; i < _size_each_state; i++)
  {
    goal_vector.fill(i,0) = -state_for_ends(i);
  }
  thread_to_state(trajectory.back(), state_for_ends);
  weight_state(state_for_ends);
  for (int i=0; i < _size_each_state; i++)
  {
    goal_vector.fill((_num_threads-2)*_size_each_state + i,0) = state_for_ends(i);
  }

  Matrix_To_File(goal_vector, FILENAME_GOALVEC);


  //vector to contain the new states
  //VectorXd new_states((_num_threads-2)*_size_each_state + (_num_threads-1)*_size_each_control);


  for (int opt_iter=0; opt_iter < num_opts; opt_iter++)
  {
    add_transitions_alltrans(trajectory);
    SparseMatrix<double> all_trans_sparse(_all_trans);
    Matrix_To_File(all_trans_sparse, FILENAME_ALLTRANS);
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

/*
    for (int i=1; i < trajectory.size()-1; i++)
    {
      VectorXd to_copy = new_states.segment(_size_each_state*(i-1), _size_each_state);
      trajectory[i]->copy_data_from_vector(to_copy);
      trajectory[i]->unviolate_total_length_constraint();
      trajectory[i]->project_length_constraint();
      trajectory[i]->minimize_energy();
    }
    */
      
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
  MatrixXd trans(_size_each_state, _size_each_control);
  for (int i=0; i < trajectory.size()-1; i++)
  {
    estimate_transition_matrix_noEdges_withTwist(trajectory[i], trans, START_AND_END);

    int num_rows_start = i*_size_each_state;
    int num_cols_start = _cols_all_unknown_states+i*_size_each_control;
    for (int r=0; r < _size_each_state; r++)
    {
      for (int c=0; c < _size_each_control; c++)
      {
        _all_trans.coeffRef(num_rows_start+r, num_cols_start+c) = trans(r,c);
      }
    }
  }
}




void thread_to_state(const Thread* thread, VectorXd& state)
{
  const int num_pieces = thread->num_pieces();
  state.resize(6*num_pieces-3+1);
  for (int piece_ind=0; piece_ind < thread->num_pieces(); piece_ind++)
  {
    state.segment(piece_ind*3, 3) = thread->vertex_at_ind(piece_ind);
  }
  state(3*num_pieces) = thread->end_angle();
}


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
}

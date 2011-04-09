#include "linearization_utils.h"



void applyControl(Thread* start, const VectorXd& u, const movement_mode movement) { 
  vector<Two_Motions*> tmp;
  applyControl(start, u, tmp, movement);

}

void applyControl(Thread* start, const VectorXd& u, vector<Two_Motions*>& motions, const movement_mode movement) {

  double max_ang;
  if (movement == START_AND_END)
  {
    double max_ang_start = max( max(abs(u(3)), abs(u(4))), abs(u(5)));
    double max_ang_end = max( max(abs(u(9)), abs(u(10))), abs(u(11)));
    max_ang = max(max_ang_start, max_ang_end);
  } else {
    double max_ang = max( max(abs(u(3)), abs(u(4))), abs(u(5)));
  }

  int number_steps = max ((int)ceil(max_ang / (M_PI/4.0)), 1);
  VectorXd u_to_use = u/((double)number_steps);

  Two_Motions* toMove;

  if (movement == START_AND_END)
  {
    Vector3d translation_start;
    translation_start << u_to_use(0), u_to_use(1), u_to_use(2);
    Vector3d translation_end;
    translation_end << u_to_use(6), u_to_use(7), u_to_use(8);

    Matrix3d rotation_start;
    rotation_from_euler_angles(rotation_start, u_to_use(3), u_to_use(4), u_to_use(5));
    Matrix3d rotation_end;
    rotation_from_euler_angles(rotation_end, u_to_use(9), u_to_use(10), u_to_use(11));

    // apply the control u to thread start, and return the new config in res
    toMove = new Two_Motions(translation_start, rotation_start, translation_end, rotation_end);
  } else {
    Vector3d translation;
    translation << u_to_use(0), u_to_use(1), u_to_use(2);

    Matrix3d rotation;
    rotation_from_euler_angles(rotation, u_to_use(3), u_to_use(4), u_to_use(5));

    // apply the control u to thread start, and return the new config in res
    if (movement == START)
      toMove = new Two_Motions(translation, rotation, Vector3d::Zero(), Matrix3d::Identity());
    else
      toMove = new Two_Motions(Vector3d::Zero(), Matrix3d::Identity(), translation, rotation);
  }


  for (int i=0; i < number_steps; i++)
  {
    
    /*pos_start = start->start_pos();
    rot_start = start->start_rot();
    pos_end = start->end_pos();
    rot_end = start->end_rot();
    toMove->_start.applyMotion(pos_start, rot_start);
    toMove->_end.applyMotion(pos_end, rot_end);
    Vector3d pointA = pos_start+rot_start.col(0)*start->rest_length();
    Vector3d pointB = pos_end-rot_end.col(0)*start->rest_length();

    if ((pointA - pointB).norm() < start->total_length() - 2*start->rest_length()) { 
      start->set_constraints(pos_start, rot_start, pos_end, rot_end);
      motions.push_back(toMove);
    }
*/

    start->apply_motion_nearEnds(*toMove);
  }

  start->minimize_energy(); 
}

  
void computeDifference(Thread* start, const Thread* goal, VectorXd& res) {
  for (int piece_ind=0; piece_ind < goal->num_pieces(); piece_ind++)
  {
    res.segment(piece_ind*3, 3) = (goal->vertex_at_ind(piece_ind) - start->vertex_at_ind(piece_ind));
  }
  for (int piece_ind=0; piece_ind < goal->num_pieces(); piece_ind++)
  {
    res.segment(goal->num_pieces()*3 +piece_ind*3, 3) = (goal->edge_at_ind(piece_ind) - start->edge_at_ind(piece_ind));
  }

}

void computeDifference_maxMag(Thread* start, const Thread* goal, VectorXd& res, double maxMag) {
  for (int piece_ind=0; piece_ind < goal->num_pieces(); piece_ind++)
  {
    res.segment(piece_ind*3, 3) = (goal->vertex_at_ind(piece_ind) - start->vertex_at_ind(piece_ind));
  }
  for (int piece_ind=0; piece_ind < goal->num_edges(); piece_ind++)
  {
    res.segment(goal->num_pieces()*3 +piece_ind*3, 3) = (goal->edge_at_ind(piece_ind) - start->edge_at_ind(piece_ind));
  }
  
  double res_norm = res.norm();
  if (res_norm > maxMag)
    res *= maxMag/res_norm;

}

void solveLinearizedControl(Thread* start, const Thread* goal, vector<Two_Motions*>& motions, const movement_mode movement) {
  //const double MAX_STEP = 2.0;
  const double DAMPING_CONST_POINTS = 0.1;
  const double DAMPING_CONST_ANGLES = 0.4;
  const double MAX_MAG = 3.0;

  int num_controls;
  if (movement == START_AND_END)
    num_controls = 12;
  else
    num_controls = 6;


  int num_pieces = start->num_pieces();
  int num_edges = start->num_edges();
  // linearize the controls around the current thread (quasistatic, no dynamics)
  MatrixXd B(6*num_pieces,num_controls);
  estimate_transition_matrix(start, B, movement);

  //weight matrix for different aspects of state
  MatrixXd weighting_mat = MatrixXd::Identity(6*num_pieces, 6*num_pieces);
  for (int i=0; i < 3*num_pieces; i++)
  {
    weighting_mat(i,i) = WEIGHT_VERTICES;
  } 
  for (int i=0; i < 3*num_edges; i++)
  {
    weighting_mat(i+3*num_pieces,i+3*num_pieces) = WEIGHT_EDGES;
  } 

  // solve the least-squares problem
  VectorXd dx(num_pieces*6);
  computeDifference_maxMag(start, goal, dx, MAX_MAG);
  VectorXd u(num_controls);
  u = B.transpose()*weighting_mat*dx;

  MatrixXd damping_mat = MatrixXd::Identity(num_controls, num_controls);
  for (int i=0; i < num_controls; i++)
  {
    if ( (i%6) < 3)
      damping_mat(i,i) = DAMPING_CONST_POINTS;
    else
      damping_mat(i,i) = DAMPING_CONST_ANGLES;
  }

  (B.transpose()*weighting_mat*B + damping_mat).llt().solveInPlace(u);
  // project it down to small step size
  //double u_norm = (B*u).norm();
  //u *= min(10.0/u_norm, u_norm/10.0);
  //u /= (u_norm > MAX_STEP ? u_norm/MAX_STEP : 1);

  // apply the given control
  applyControl(start, u, motions, movement);
}

void solveLinearizedControl(Thread* start, const Thread* goal, const movement_mode movement) {
  vector<Two_Motions*> tmp;
  solveLinearizedControl(start, goal, tmp, movement);
}



void estimate_transition_matrix(Thread* thread, MatrixXd& A, const movement_mode movement)
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
    
    du(i) = eps;
    thread->restore_thread_pieces(thread_backup_pieces);
    applyControl(thread, du, movement);
    for (int piece_ind=0; piece_ind < num_pieces; piece_ind++)
    {
      A.block(piece_ind*3, i, 3,1) = thread->vertex_at_ind(piece_ind);
    }
    for (int piece_ind=0; piece_ind < num_edges; piece_ind++)
    {
      A.block(3*num_pieces + piece_ind*3, i, 3,1) = thread->edge_at_ind(piece_ind);
    }

    du(i) = -eps;
    thread->restore_thread_pieces(thread_backup_pieces);
    applyControl(thread, du, movement);
    for (int piece_ind=0; piece_ind < num_pieces; piece_ind++)
    {
      A.block(piece_ind*3, i, 3,1) -= thread->vertex_at_ind(piece_ind);
    }
    for (int piece_ind=0; piece_ind < num_edges; piece_ind++)
    {
      A.block(3*num_pieces + piece_ind*3, i, 3,1) -= thread->edge_at_ind(piece_ind);
    }
  }
  A /= 2.0*eps;
  thread->restore_thread_pieces(thread_backup_pieces);
}






void iterative_control_opt(vector<Thread*>& trajectory, vector<VectorXd>& controls)
{
  const int size_each_state = 6*trajectory.front()->num_edges() + 3;
  const int size_each_control = 12;
  SparseMatrix<double, Eigen::RowMajor> all_trans_mat(size_each_state*(trajectory.size()-1), (trajectory.size()-2)*size_each_state + (trajectory.size()-1)*size_each_control);
  
  ico_compute_massive_trans(trajectory, all_trans_mat);

}

void ico_compute_massive_trans(vector<Thread*>& trajectory, SparseMatrix<double, RowMajor>& massive_mat)
{
  const int num_verts = trajectory.front()->num_pieces();
  const int num_edges = trajectory.front()->num_edges();
  const int size_each_state = 6*num_edges + 3;
  const int size_each_control = 12;
  const int cols_all_unknown_states = (trajectory.size()-2)*size_each_state;

  MatrixXd trans(size_each_state, size_each_control);
  //weight matrix for different aspects of state
  MatrixXd state_weight_mat = MatrixXd::Identity(6*num_verts, 6*num_verts);
  for (int i=0; i < 3*num_verts; i++)
  {
    state_weight_mat(i,i) = WEIGHT_VERTICES;
  } 
  for (int i=0; i < 3*num_edges; i++)
  {
    state_weight_mat(i+3*num_verts,i+3*num_verts) = WEIGHT_EDGES;
  } 

  std::cout << "size: " << massive_mat.rows() << " " << massive_mat.cols() << std::endl;

  massive_mat.startFill();
  for (int i=0; i < trajectory.size()-1; i++)
  {
    std::cout << "i: " << i << std::endl;
    estimate_transition_matrix(trajectory[i], trans, START_AND_END);

    int num_rows_start = i*size_each_state;
    int num_cols_start = cols_all_unknown_states+i*size_each_control;
    for (int r=0; r < size_each_state; r++)
    {
      std::cout << "r: " << r << std::endl;
      if (i != 0)
      {
        std::cout << "ind: " << (num_rows_start+r) <<" " << (num_rows_start+r -size_each_state) << std::endl;
        massive_mat.fill(num_rows_start+r,num_rows_start+r -size_each_state) = state_weight_mat(r,r);
      }
      if (i != trajectory.size()-2)
      {
        std::cout << "ind: " << (num_rows_start+r) <<" " << (num_rows_start+r) << std::endl;
        massive_mat.fill(num_rows_start+r,num_rows_start+r) = -state_weight_mat(r,r);
      }

      for (int c=0; c < size_each_control; c++)
      {
        std::cout << "ind: " << (num_rows_start+r) <<" " << (num_cols_start+c) << std::endl;
        massive_mat.fill(num_rows_start+r, num_cols_start+c) = trans(r,c);
      }
    }
  }
  massive_mat.endFill();


  std::cout << "huge shit:\n" << massive_mat << std::endl;
  
}







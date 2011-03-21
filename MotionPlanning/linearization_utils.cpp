#include "linearization_utils.h"


void applyControl(Thread* start, const VectorXd& u) {
 // if (u.squaredNorm() < 1e-5)
  //  return;
  double max_ang = max( max(abs(u(3)), abs(u(4))), abs(u(5)));
  int number_steps = max ((int)ceil(max_ang / (M_PI/4.0)), 1);
  VectorXd u_to_use = u/((double)number_steps);

  Vector3d translation;
  translation << u_to_use(0), u_to_use(1), u_to_use(2);

  /*
  double dw = 1.0 - u(3)*u(3) - u(4)*u(4) - u(5)*u(5);
  if (dw < 0) {
    cout << "Huge differential quaternion: " << endl;
  }
  dw = max(dw, 0.01);
  dw = sqrt(dw);
  Eigen::Quaterniond q(dw, u(3),u(4),u(5));
  Matrix3d rotation(q);
*/  

  Matrix3d rotation;
  rotation_from_euler_angles(rotation, u_to_use(3), u_to_use(4), u_to_use(5));


  // apply the control u to thread start, and return the new config in res
  Frame_Motion toMove(translation, rotation);


  for (int i=0; i < number_steps; i++)
  {
    Vector3d end_pos = start->end_pos();
    Matrix3d end_rot = start->end_rot();
    toMove.applyMotion(end_pos, end_rot);
    start->set_end_constraint(end_pos, end_rot);
  }

  start->minimize_energy();
}

void applyControl(Thread* start, const VectorXd& u, VectorXd* res) { 
  int N = start->num_pieces();
  res->setZero(3*N);
  applyControl(start, u); 
  start->toVector(res); 
}

  
void computeDifference(Thread* start, Thread* goal, VectorXd& res) {
  for (int piece_ind=0; piece_ind < goal->num_pieces(); piece_ind++)
  {
    res.segment(piece_ind*3, 3) = goal->vertex_at_ind(piece_ind) - start->vertex_at_ind(piece_ind);
  }
}

void computeDifference_maxMag(Thread* start, Thread* goal, VectorXd& res, double maxMag) {
  for (int piece_ind=0; piece_ind < goal->num_pieces(); piece_ind++)
  {
    res.segment(piece_ind*3, 3) = goal->vertex_at_ind(piece_ind) - start->vertex_at_ind(piece_ind);
  }
  
  double res_norm = res.norm();
  if (res_norm > maxMag)
    res *= maxMag/res_norm;

}

void solveLinearizedControl(Thread* start, Thread* goal) {
  //const double MAX_STEP = 2.0;
  const double DAMPING_CONST = 0.2;
  const double MAX_MAG = 7.0;
  
  // linearize the controls around the current thread (quasistatic, no dynamics)
  int num_pieces = start->num_pieces();
  MatrixXd B(3*num_pieces,6);

  estimate_transition_matrix(start, B);

  // solve the least-squares problem
  VectorXd dx(num_pieces*3);
  //computeDifference_maxMag(start, goal, dx, MAX_MAG);
  computeDifference_maxMag(start, goal, dx, MAX_MAG);
  VectorXd u = B.transpose()*dx;
  
  (B.transpose()*B + DAMPING_CONST*MatrixXd::Identity(6, 6)).llt().solveInPlace(u);
  // project it down to small step size
  //double u_norm = (B*u).norm();
  //u *= min(10.0/u_norm, u_norm/10.0);
  //u /= (u_norm > MAX_STEP ? u_norm/MAX_STEP : 2);

  
  //u /= (u_norm > MAX_STEP ? u_norm/MAX_STEP : 1);

  //std::cout << u.transpose() << std::endl;

  // apply the given control
  applyControl(start, u);
}


void estimate_transition_matrix(Thread* thread, MatrixXd& A)
{
  int num_pieces = thread->num_pieces();
  vector<ThreadPiece*> thread_backup_pieces;
  thread->save_thread_pieces_and_resize(thread_backup_pieces);

  VectorXd du(6);
  const double eps = 1e-4;
  for(int i = 0; i < 6; i++)
  {
    du.setZero();
    
    du(i) = eps;
    thread->restore_thread_pieces(thread_backup_pieces);
    applyControl(thread, du);
    for (int piece_ind=0; piece_ind < num_pieces; piece_ind++)
    {
      A.block(piece_ind*3, i, 3,1) = thread->vertex_at_ind(piece_ind);
    }

    du(i) = -eps;
    thread->restore_thread_pieces(thread_backup_pieces);
    applyControl(thread, du);
    for (int piece_ind=0; piece_ind < num_pieces; piece_ind++)
    {
      A.block(piece_ind*3, i, 3,1) -= thread->vertex_at_ind(piece_ind);
    }
  }
  A /= 2.0*eps;
  thread->restore_thread_pieces(thread_backup_pieces);
}


#include "linearization_utils.h"
#include <boost/progress.hpp>


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
  MatrixXd B(6*num_pieces-3,num_controls);
  estimate_transition_matrix(start, B, movement);

  //weight matrix for different aspects of state
  MatrixXd weighting_mat = MatrixXd::Identity(6*num_pieces-3, 6*num_pieces-3);
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


void simpleInterpolation(Thread* start, Thread* goal, vector<Two_Motions*>& motions) {

  double step = 1; 

  Vector3d cur_start_pos = start->start_pos();
  Matrix3d cur_start_rot = start->start_rot();
  Vector3d cur_end_pos = start->end_pos();
  Matrix3d cur_end_rot = start->end_rot();

  Vector3d goal_start_pos = goal->start_pos();
  Matrix3d goal_start_rot = goal->start_rot();
  Vector3d goal_end_pos = goal->end_pos();
  Matrix3d goal_end_rot = goal->end_rot();

  Eigen::Quaterniond cur_startq(cur_start_rot);
  Eigen::Quaterniond cur_endq(cur_end_rot);
  Eigen::Quaterniond goal_startq(goal_start_rot);
  Eigen::Quaterniond goal_endq(goal_end_rot); 

  Vector3d cur_start_col = cur_start_rot*Vector3d::UnitX();
  Vector3d cur_end_col = cur_end_rot*Vector3d::UnitX();
  Vector3d goal_start_col = goal_start_rot*Vector3d::UnitX();
  Vector3d goal_end_col = goal_end_rot*Vector3d::UnitX();

  double diff_start_angle = angle_between(cur_start_col, goal_start_col);
  double diff_end_angle = angle_between(cur_end_col, goal_end_col);
  double t_start = M_PI/8.0;
  double t_end = M_PI/8.0;

  Eigen::Quaterniond final_startq = cur_startq.slerp(t_start, goal_startq).normalized();
  Eigen::Quaterniond final_endq = cur_endq.slerp(t_end, goal_endq).normalized();
  
  Matrix3d interpolated_start_rotation = (final_startq*cur_startq.inverse()).toRotationMatrix(); 
  Matrix3d interpolated_end_rotation = (final_endq*cur_endq.inverse()).toRotationMatrix();
  Vector3d interpolated_start_pos = goal_start_pos - cur_start_pos;
  Vector3d interpolated_end_pos = goal_end_pos - cur_end_pos;

  if(interpolated_start_pos.squaredNorm() > 0) { 
    interpolated_start_pos.normalized();
    interpolated_start_pos *= step;
  }
  
  if(interpolated_end_pos.squaredNorm() > 0) { 
    interpolated_end_pos.normalized();
    interpolated_end_pos *= step;
  }

  Two_Motions* interpMotion = new Two_Motions(interpolated_start_pos, interpolated_start_rotation, interpolated_end_pos, interpolated_end_rotation);

  motions.push_back(interpMotion);
  start->apply_motion_nearEnds(*interpMotion);

}

void interpolateThreads(vector<Thread*>&traj, vector<Two_Motions*>& controls) {
  
  double T = traj.size(); 
  controls.resize(T-1);

  Thread* start = traj.front();
  Thread* end = traj[T-1];

  vector<Vector3d> start_pts;
  vector<Vector3d> end_pts;
  vector<double> start_twist_angles;
  vector<double> end_twist_angles; 
  start->get_thread_data(start_pts, start_twist_angles);  
  end->get_thread_data(end_pts, end_twist_angles); 
  
  Matrix3d start_start_rot = start->start_rot();
  Matrix3d end_start_rot = end->start_rot(); 
  Matrix3d start_end_rot = start->end_rot();
  Matrix3d end_end_rot = end->end_rot(); 

  Eigen::Quaterniond start_startq(start_start_rot);
  Eigen::Quaterniond end_startq(end_start_rot);
  Eigen::Quaterniond start_endq(start_end_rot);
  Eigen::Quaterniond end_endq(end_end_rot); 

  cout << "starting interpolation" << endl; 
  
  for (int t = 1; t < T-1; t++) 
  {
    Vector3d ctrl_start_translation = Vector3d::Zero(); 
    Vector3d ctrl_end_translation = Vector3d::Zero(); 
    Matrix3d ctrl_start_rot; 
    Matrix3d ctrl_end_rot;
    vector<Vector3d> prevPts; 
    vector<Vector3d> interpolated_pts;
    vector<double> interpolated_angles; 
    for (int p = 0; p < start_pts.size(); p++)
    { 
      Vector3d pts = ((T-t)/T)*start_pts[p] + (t/T)*end_pts[p];
      interpolated_pts.push_back(pts);
      interpolated_angles.push_back(0.0);
    }
    //if (t == 1) prevPts = start_pts;  

    //ctrl_start_translation = interpolated_pts[0] - prevPts[0]; 
    //ctrl_end_translation = interpolated_pts[start_pts.size()-1] - prevPts[start_pts.size()-1]; 

    Eigen::Quaterniond interp_start_q = start_startq.slerp(t/T, end_startq);
    Eigen::Quaterniond interp_end_q = start_endq.slerp(t/T, end_endq);
    Matrix3d start_rot = interp_start_q.toRotationMatrix();
    Matrix3d end_rot = interp_end_q.toRotationMatrix();
    
    //ctrl_start_rot = (interp_start_q*start_startq.inverse()).toRotationMatrix();
    //ctrl_end_rot = (interp_end_q*start_endq.inverse()).toRotationMatrix(); 

    Thread* interpolated_thread = new Thread(interpolated_pts, interpolated_angles, start_rot, end_rot);
    traj[t] = interpolated_thread;

    //prevPts =interpolated_pts; 
  }
  boost::progress_display progress(T-2); 

  #pragma omp parallel for num_threads(NUM_CPU_THREADS) 
  for (int t = 1; t < traj.size()-1; t++) { 
    traj[t]->minimize_energy(); 
    ++progress; 
  }
}


void estimate_transition_matrix_noEdges_withTwist(Thread* thread, MatrixXd& A, const movement_mode movement)
{
  const int num_pieces = thread->num_pieces();
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
    A(3*num_pieces, i) = thread->end_angle();

    du(i) = -eps;
    thread->restore_thread_pieces(thread_backup_pieces);
    applyControl(thread, du, movement);
    for (int piece_ind=0; piece_ind < num_pieces; piece_ind++)
    {
      A.block(piece_ind*3, i, 3,1) -= thread->vertex_at_ind(piece_ind);
    }
    A(3*num_pieces, i) -= thread->end_angle();
  }
  A /= 2.0*eps;
  thread->restore_thread_pieces(thread_backup_pieces);
}


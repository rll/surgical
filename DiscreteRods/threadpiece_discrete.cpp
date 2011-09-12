#include "threadpiece_discrete.h"
#include "thread_discrete.h"
#include "EnvObjects/World.h"
#include "Collisions/CollisionWorld.h"

ThreadPiece::ThreadPiece(bool add_col_obj)
	: Object(THREAD_PIECE)
	, rot(Matrix3d::Zero())
	, _my_thread(NULL)
	, _piece_ind(-1)
{
	grad_offsets[0] = Vector3d(grad_eps, 0.0, 0.0);
	grad_offsets[1] = Vector3d(0.0, grad_eps, 0.0);
	grad_offsets[2] = Vector3d(0.0, 0.0, grad_eps);
	rot = Matrix3d::Zero();
	
	_rest_length = 0.0;
	
	if (add_col_obj) {
		btCapsuleShapeX* capsule_piece = new btCapsuleShapeX(btScalar(THREAD_RADIUS+REPULSION_DIST), btScalar(_rest_length));
		capsule_piece->setMargin(0.f);
		_col_obj = new btCollisionObject();
		_col_obj->setCollisionShape(capsule_piece);
		_col_obj->setUserPointer(this);
		if (_my_thread->world->collision_world != NULL)
			_my_thread->world->collision_world->addCollisionObject(_col_obj);
	} else {
		_col_obj = NULL;
	}
}

ThreadPiece::ThreadPiece(const Vector3d& vertex, const double angle_twist, const double rest_length, Thread* my_thread, bool add_col_obj)
	: Object(THREAD_PIECE)
	, _vertex(vertex)
	, _angle_twist(angle_twist)
	, _rest_length(rest_length)
	, _prev_piece(NULL)
	, _next_piece(NULL), rot(Matrix3d::Zero())
	, _my_thread(my_thread)
	, _piece_ind(-1)
{
	grad_offsets[0] = Vector3d(grad_eps, 0.0, 0.0);
	grad_offsets[1] = Vector3d(0.0, grad_eps, 0.0);
	grad_offsets[2] = Vector3d(0.0, 0.0, grad_eps);
	
	if (add_col_obj) {
		btCapsuleShapeX* capsule_piece = new btCapsuleShapeX(btScalar(THREAD_RADIUS+REPULSION_DIST), btScalar(_rest_length));
		capsule_piece->setMargin(0.f);
		_col_obj = new btCollisionObject();
		_col_obj->setCollisionShape(capsule_piece);
		_col_obj->setUserPointer(this);
		if (_my_thread->world->collision_world != NULL)
			_my_thread->world->collision_world->addCollisionObject(_col_obj);
	} else {
		_col_obj = NULL;
	}
}

ThreadPiece::ThreadPiece(const Vector3d& vertex, const double angle_twist, const double rest_length, ThreadPiece* prev, ThreadPiece* next, Thread* my_thread, bool add_col_obj)
	: Object(THREAD_PIECE)
	, _vertex(vertex)
	, _angle_twist(angle_twist)
	, _rest_length(rest_length)
	, rot(Matrix3d::Zero())
	, _my_thread(my_thread)
	, _piece_ind(-1)
{
	grad_offsets[0] = Vector3d(grad_eps, 0.0, 0.0);
	grad_offsets[1] = Vector3d(0.0, grad_eps, 0.0);
	grad_offsets[2] = Vector3d(0.0, 0.0, grad_eps);
  set_prev(prev);
  set_next(next);

 	if (add_col_obj) {
		btCapsuleShapeX* capsule_piece = new btCapsuleShapeX(btScalar(THREAD_RADIUS+REPULSION_DIST), btScalar(_rest_length));
		capsule_piece->setMargin(0.f);
		_col_obj->setCollisionShape(capsule_piece);
		_col_obj->setUserPointer(this);
		if (_my_thread->world->collision_world != NULL)
			_my_thread->world->collision_world->addCollisionObject(_col_obj);
	} else {
		_col_obj = NULL;
	}
}


ThreadPiece::ThreadPiece(const ThreadPiece& rhs, bool add_col_obj)
	: Object(THREAD_PIECE)
	, _vertex(rhs._vertex)
	, _angle_twist(rhs._angle_twist)
	, _rest_length(rhs._rest_length)
	, _edge(rhs._edge)
	, _edge_norm(rhs._edge_norm)
	, _curvature_binormal(rhs._curvature_binormal)
	, _bishop_frame(rhs._bishop_frame)
	, _material_frame(rhs._material_frame)
	, _prev_piece(rhs._prev_piece)
	, _next_piece(rhs._next_piece)
	, _my_thread(rhs._my_thread)
	, _piece_ind(-1)
{
	grad_offsets[0] = Vector3d(grad_eps, 0.0, 0.0);
	grad_offsets[1] = Vector3d(0.0, grad_eps, 0.0);
	grad_offsets[2] = Vector3d(0.0, 0.0, grad_eps);
	rot = Matrix3d::Zero();
	
	if (add_col_obj) {
		btCapsuleShapeX* capsule_piece = new btCapsuleShapeX(btScalar(THREAD_RADIUS+REPULSION_DIST), btScalar(_rest_length));
		capsule_piece->setMargin(0.f);
		_col_obj = new btCollisionObject();
		_col_obj->setCollisionShape(capsule_piece);
		_col_obj->setUserPointer(this);
		if (_my_thread->world->collision_world != NULL)
			_my_thread->world->collision_world->addCollisionObject(_col_obj);
	} else {
		_col_obj = NULL;
	}
}
	
ThreadPiece::ThreadPiece(const ThreadPiece& rhs, Thread* my_thread, bool add_col_obj)
	: Object(THREAD_PIECE)
	, _vertex(rhs._vertex)
	, _angle_twist(rhs._angle_twist)
	, _rest_length(rhs._rest_length)
	, _edge(rhs._edge)
	, _edge_norm(rhs._edge_norm)
	, _curvature_binormal(rhs._curvature_binormal)
	, _bishop_frame(rhs._bishop_frame)
	, _material_frame(rhs._material_frame)
	, _prev_piece(rhs._prev_piece)
	, _next_piece(rhs._next_piece)
	, _my_thread(my_thread)
	, _piece_ind(-1)
{
	grad_offsets[0] = Vector3d(grad_eps, 0.0, 0.0);
	grad_offsets[1] = Vector3d(0.0, grad_eps, 0.0);
	grad_offsets[2] = Vector3d(0.0, 0.0, grad_eps);
	rot = Matrix3d::Zero();
	
	if (add_col_obj) {
		btCapsuleShapeX* capsule_piece = new btCapsuleShapeX(btScalar(THREAD_RADIUS+REPULSION_DIST), btScalar(_rest_length));
		capsule_piece->setMargin(0.f);
		_col_obj = new btCollisionObject();
		_col_obj->setCollisionShape(capsule_piece);
		_col_obj->setUserPointer(this);
		if (_my_thread->world->collision_world != NULL)
			_my_thread->world->collision_world->addCollisionObject(_col_obj);
	} else {
		_col_obj = NULL;
	}
}

ThreadPiece::~ThreadPiece()
{
	if (_col_obj != NULL) {
		_my_thread->world->collision_world->removeCollisionObject(_col_obj);
		delete _col_obj;
	}
}

void ThreadPiece::set_vertex(const Vector3d& vertex)
{
  _vertex = vertex;
}

double ThreadPiece::energy()
{
  return energy_curvature() + energy_twist() + energy_grav() + energy_stretch();
}

//not defined for first or last piece
double ThreadPiece::energy_curvature()
{
  if (_prev_piece == NULL || _next_piece == NULL)
    return 0.0;

#ifdef ISOTROPIC
	return (BEND_COEFF*_curvature_binormal.squaredNorm())/(_prev_piece->_rest_length + _rest_length);
#else
  //find first centerline curvature
  Vector2d curve_with_prev(_curvature_binormal.dot(_prev_piece->_material_frame.col(2)), -_curvature_binormal.dot(_prev_piece->_material_frame.col(1)) );

  Vector2d curve_with_after(_curvature_binormal.dot((_material_frame).col(2)), -_curvature_binormal.dot((_material_frame).col(1)) );


  return (curve_with_prev.dot(B*curve_with_prev)+ curve_with_after.dot(B*curve_with_after)) / (2.0*(_prev_piece->_rest_length + _rest_length));	//check: what about transpose and minus?
#endif
}

//not defined for first or last piece
//inefficient for isotropic
double ThreadPiece::energy_twist()
{
#ifdef ISOTROPIC
 // std::cout << "SHOULD NOT BE CALLING THIS - very inefficient" << std::endl;
  if (_prev_piece == NULL || _next_piece == NULL) //check || _next_piece->_next_piece == NULL)
  {
    return 0.0; //same - since the first twist is set to be zero (aligning bishop to initial material frame)
  } else {
    double angle_diff = angle_twist() - _prev_piece->angle_twist();    
    return ( (TWIST_COEFF*angle_diff*angle_diff)/(_prev_piece->_rest_length + _rest_length) );
  }

#else
  if (_prev_piece == NULL || _next_piece == NULL)
  {
    return 0.0; //same - since the first twist is set to be zero (aligning bishop to initial material frame)
  } else {
    double angle_diff = angle_twist() - _prev_piece->angle_twist();    
    return ( (TWIST_COEFF*angle_diff*angle_diff)/(_prev_piece->_rest_length + _rest_length) );
  }
#endif
}

//not defined for first or last piece
double ThreadPiece::energy_stretch()
{
  if (STRETCH_COEFF<=0)
  	return 0.0;
  if (_next_piece == NULL)
    return 0.0;
  //std::cout << "energy stretch: " << STRETCH_COEFF*(edge_after.norm() - (my_thread->rest_length())) << std::endl;
  return STRETCH_COEFF/2.0 * pow(_edge_norm/_rest_length - 1.0, 2.0) * _rest_length;

  //double dl = abs(_edge_norm-_rest_length);
//  double a = 0.05*_rest_length;
//  double dist = 2.0*a - abs(_edge_norm-_rest_length);
//	//cout << "diff: " << a << "\t\t" << abs(_edge_norm-_rest_length) << endl;
//	if (dist < 0) {	//abs(_edge_norm-_rest_length) > 2.0*a
//		//cout << "case 0" << endl;
//		return 0.0; //STRETCH_COEFF * 10000.0;
//	} else if (dist < a) { //abs(_edge_norm-_rest_length) > a
//		//cout << "case 1" << endl;
//		return STRETCH_COEFF * (-log(dist/a) + 0.5);
//	} else if (dist < 2.0*a) {	//abs(_edge_norm-_rest_length) > 0
//		//cout << "case 2" << endl;
//		return STRETCH_COEFF * (dist*dist/(a*a*2) - 2*dist/a + 2);
//	} else {	//abs(_edge_norm-_rest_length) == 0
//		//cout << "case 3" << endl;
//		return 0.0;
//	}
}

double ThreadPiece::energy_grav()
{
	return _vertex(2)*GRAV_COEFF;
}


void ThreadPiece::set_prev(ThreadPiece* prev)
{
  _prev_piece = prev;
}

void ThreadPiece::set_next(ThreadPiece* next)
{
  _next_piece = next;
}


void ThreadPiece::set_bend_coeff(double bend_coeff)
{
	BEND_COEFF = bend_coeff;
	B = Matrix2d::Identity()*BEND_COEFF;
	JB = J*B;
}

void ThreadPiece::set_bend_matrix(const Matrix2d& bend_matrix)
{
	B = bend_matrix;
	JB = J*B;
}

void ThreadPiece::set_twist_coeff(double twist_coeff)
{
	TWIST_COEFF = twist_coeff;
}

void ThreadPiece::set_grav_coeff(double grav_coeff)
{
	GRAV_COEFF = grav_coeff;
}

double ThreadPiece::get_bend_coeff() {return BEND_COEFF;}
Matrix2d ThreadPiece::get_bend_matrix() {return B;}
double ThreadPiece::get_twist_coeff() {return TWIST_COEFF;}
double ThreadPiece::get_grav_coeff() {return GRAV_COEFF;}

void ThreadPiece::gradient_twist(double& grad)
{
  grad = -(_next_piece->_angle_twist - _angle_twist)/(_rest_length + _next_piece->_rest_length);

  if (_prev_piece != NULL)
  {
    grad += (_angle_twist - _prev_piece->_angle_twist)/(_prev_piece->_rest_length + _rest_length);
  } else {
  	cout << "Internal error: ThreadPiece::gradient_twist(): thread_piece should not be the first one." << endl;
    grad += (_angle_twist)/(2.0*(_rest_length));
  }
  grad *= 2.0*TWIST_COEFF;


  //non-isotropic, so affects curvature energy too
  Vector2d w_j(_curvature_binormal.dot(_material_frame.col(2)), -_curvature_binormal.dot(_material_frame.col(1)) );
  Vector2d w_j_1(_next_piece->_curvature_binormal.dot(_material_frame.col(2)), -_next_piece->_curvature_binormal.dot(_material_frame.col(1)) );

  if (_prev_piece != NULL)
  {
    grad += (w_j.dot(JB*w_j))/(_prev_piece->_rest_length + _rest_length);	// check: shouldn't the second w_j be transposed? and what about -w_j bar?
  }
  grad += (w_j_1.dot(JB*w_j_1))/(_rest_length + _next_piece->_rest_length);	//check: same as above


}

void ThreadPiece::stretch_gradient_vertex(Vector3d& grad) {
	if (STRETCH_COEFF > 0.0) {
		double a = 0.05*_rest_length;
		double dist;
		double dl;
		Vector3d direction;
		
		if (_prev_piece != NULL && _prev_piece->_prev_piece != NULL) { //(piece_ind > 1)
			//vertex i related to edge of the prev piece
			dist = 2.0*a - abs(_prev_piece->_edge_norm-_prev_piece->_rest_length);
			dl = _prev_piece->_edge_norm-_prev_piece->_rest_length;
			// if stretching, dl > 0 and direction should be _prev_piece->_edge/_prev_piece->_edge_norm
			direction = _prev_piece->_edge/_prev_piece->_edge_norm;
			if (dl > 0) direction *= -1.0;
	
			if (dist < 0) {
				//grad -= - STRETCH_COEFF * 1000.0 * abs(dist) * direction;
			} else if (dist < a) {
				grad -= STRETCH_COEFF * (1/dist) * direction;
			} else if (dist < 2.0*a) {
				grad -= - STRETCH_COEFF * (dist/(a*a) - 2/a) * direction;
			}
		}
		
		if (_next_piece != NULL && _next_piece->_next_piece != NULL) { //piece_ind < _my_thread->_thread_pieces.size()-2)
			//vertex i related to edge of this piece
			dist = 2.0*a - abs(_edge_norm-_rest_length);
			dl = _edge_norm-_rest_length;
			// if stretching, dl > 0 and direction should be -_edge/_edge_norm
			direction = _edge/_edge_norm;
			if (dl < 0) direction *= -1.0;
	
			if (dist < 0) {
				//grad -= - STRETCH_COEFF * 1000.0 * abs(dist) * direction;
			} else if (dist < a) {
				grad -= STRETCH_COEFF * (1/dist) * direction;
			} else if (dist < 2.0*a) {
				grad -= - STRETCH_COEFF * (dist/(a*a) - 2/a) * direction;
			}
		}
	}
}

//not defined for first 2 or last 2 pieces
void ThreadPiece::gradient_vertex(Vector3d& grad)
{
#ifdef ISOTROPIC
	double beta_angle_diff_over_L = TWIST_COEFF*(_my_thread->end_angle() - _my_thread->start_angle())/(_my_thread->total_length() - ((_my_thread->start_rest_length()+_my_thread->end_rest_length())/2.0));

	skew_i.setZero();
	skew_i_im1.setZero();

	skew_symmetric_for_cross_fast(_prev_piece->_prev_piece->_edge, skew_i_im1);
	//del_kb_i_ip1 = (2.0*skew_i_im1 - _prev_piece->_curvature_binormal*(_prev_piece->_prev_piece->_edge.transpose())) / (rest_length_squared + _prev_piece->_prev_piece->_edge.dot(_prev_piece->_edge));
	del_kb_i_ip1 = (2.0*skew_i_im1 - _prev_piece->_curvature_binormal*(_prev_piece->_prev_piece->_edge.transpose())) / ((_prev_piece->_prev_piece->_rest_length)*(_prev_piece->_rest_length) + _prev_piece->_prev_piece->_edge.dot(_prev_piece->_edge));
	del_psi_i_ip1 = -_prev_piece->_curvature_binormal/(2.0*(_prev_piece->_rest_length));
	
	grad = (2.0*BEND_COEFF/(_prev_piece->_prev_piece->_rest_length + _prev_piece->_rest_length))*del_kb_i_ip1.transpose()*_prev_piece->_curvature_binormal - beta_angle_diff_over_L*del_psi_i_ip1;


	skew_symmetric_for_cross_fast(_next_piece->_edge, skew_i);
	del_kb_i_im1 = (2.0*skew_i + _next_piece->_curvature_binormal*(_next_piece->_edge.transpose())) / ((_rest_length)*(_next_piece->_rest_length) + _edge.dot(_next_piece->_edge));
	del_psi_i_im1 = _next_piece->_curvature_binormal/(2.0*(_rest_length));

	grad += (2.0*BEND_COEFF/(_rest_length + _next_piece->_rest_length))*del_kb_i_im1.transpose()*_next_piece->_curvature_binormal - beta_angle_diff_over_L*del_psi_i_im1;


	skew_symmetric_for_cross_fast(_edge, skew_i);
	skew_symmetric_for_cross_fast(_prev_piece->_edge, skew_i_im1);

	double denom_i_i = (_prev_piece->_rest_length)*(_rest_length) + _prev_piece->_edge.dot(_edge);
	del_kb_i_im1 = (2.0*skew_i + _curvature_binormal*(_edge.transpose())) / denom_i_i;
	del_kb_i_ip1 = (2.0*skew_i_im1 - _curvature_binormal*(_prev_piece->_edge.transpose())) / denom_i_i;
	del_psi_i_im1 = _curvature_binormal/(2.0*(_prev_piece->_rest_length));
	del_psi_i_ip1 = -_curvature_binormal/(2.0*(_rest_length));
	
	grad += (2.0*BEND_COEFF/(_prev_piece->_rest_length + _rest_length))*(-del_kb_i_im1-del_kb_i_ip1).transpose()*_curvature_binormal - beta_angle_diff_over_L*(-del_psi_i_im1-del_psi_i_ip1);

	grad += Vector3d::UnitZ()*GRAV_COEFF;

	if (STRETCH_COEFF > 0.0) {
		double factor;
		if ((_next_piece != NULL && _next_piece->_next_piece != NULL && _next_piece->_next_piece->_next_piece != NULL) ||
				 (_prev_piece != NULL && _prev_piece->_prev_piece != NULL && _prev_piece->_prev_piece->_prev_piece != NULL))	//(_my_thread->_thread_pieces.size()-3) ||	piece_ind == 2)
			factor = 0.25;
		else
			factor = 1.0;
		if (_next_piece != NULL && _next_piece->_next_piece != NULL) //piece_ind < _my_thread->_thread_pieces.size()-2)
			grad -= factor * STRETCH_COEFF * (_edge_norm/_rest_length - 1.0) * _edge.normalized();
		if (_prev_piece != NULL && _prev_piece->_prev_piece != NULL) //(piece_ind > 1)
			grad += factor * STRETCH_COEFF * (_prev_piece->_edge_norm/_prev_piece->_rest_length - 1.0) * _prev_piece->_edge.normalized();
	}
#else
  grad.setZero();

  Matrix3d edge_skew_prev = Matrix3d::Zero();
  Matrix3d edge_skew_next = Matrix3d::Zero();
  Matrix3d del_kb_k;
  Matrix23d material_frame_del_w_k_j;
  Matrix23d del_w_k_j;
  Vector2d w_k_j;
  Vector3d sum_writhe(0.0, 0.0, 0.0);
  double denom_k;

  ThreadPiece* kPiece = _prev_piece;
  ThreadPiece* jPiece = kPiece->_prev_piece;
  material_frame_del_w_k_j.row(0)= jPiece->_material_frame.col(2).transpose();
  material_frame_del_w_k_j.row(1)= -jPiece->_material_frame.col(1).transpose();
  while (kPiece != _next_piece->_next_piece)
  {
  	denom_k = k_piece->prev_piece->_rest_length + k_piece->_rest_length;
    calc_del_kb_k(del_kb_k, kPiece, edge_skew_prev, edge_skew_next);

    //j=k-1
    //jPiece = kPiece->_prev_piece;
    w_k_j(0) = kPiece->_curvature_binormal.dot(jPiece->_material_frame.col(2));
    w_k_j(1) = -(kPiece->_curvature_binormal.dot(jPiece->_material_frame.col(1)));

    del_w_k_j = material_frame_del_w_k_j*del_kb_k - J*w_k_j*sum_writhe.transpose();
    grad += del_w_k_j.transpose()*B*w_k_j/denom_k;

  //  std::cout <<"wkj:\n" <<  w_k_j << std::endl;
  //  std::cout <<"del wkj:\n" <<  del_w_k_j << std::endl;
    //j=k
    jPiece = kPiece;
    w_k_j(0) = kPiece->_curvature_binormal.dot(jPiece->_material_frame.col(2));
    w_k_j(1) = -(kPiece->_curvature_binormal.dot(jPiece->_material_frame.col(1)));
    material_frame_del_w_k_j.row(0)= jPiece->_material_frame.col(2).transpose();
    material_frame_del_w_k_j.row(1)= -jPiece->_material_frame.col(1).transpose();
    add_sum_writhe(jPiece, sum_writhe);

    del_w_k_j = material_frame_del_w_k_j*del_kb_k - J*w_k_j*sum_writhe.transpose();
    grad += del_w_k_j.transpose()*B*w_k_j/denom_k;

  //  std::cout <<"wkj:\n" <<  w_k_j << std::endl;
  //  std::cout <<"del wkj:\n" <<  del_w_k_j << std::endl;

  //  std::cout << "Grad:\n" << grad << std::endl;
    kPiece = kPiece->_next_piece;
  }


  //now, loop for k=i+2 to k=n
  //only second part of derivative now matters (first always zero)
  //sum writhe also no longer changes
  while (kPiece->_next_piece != NULL)
  {
  	denom_k = k_piece->prev_piece->_rest_length + k_piece->_rest_length;
    //j = k-1
    w_k_j(0) = kPiece->_curvature_binormal.dot(kPiece->_prev_piece->_material_frame.col(2));
    w_k_j(1) = -kPiece->_curvature_binormal.dot(kPiece->_prev_piece->_material_frame.col(1));
    del_w_k_j = -J*w_k_j*sum_writhe.transpose();
    grad += del_w_k_j.transpose()*B*w_k_j / denom_k;

    //j = k
    w_k_j(0) = kPiece->_curvature_binormal.dot(kPiece->_material_frame.col(2));
    w_k_j(1) = -kPiece->_curvature_binormal.dot(kPiece->_material_frame.col(1));
    del_w_k_j= -J*w_k_j*sum_writhe.transpose();
    grad += del_w_k_j.transpose()*B*w_k_j / denom_k;

    kPiece = kPiece->_next_piece;
  }


  //add second part
  //set curr_piece to be second to last
  kPiece = kPiece->_prev_piece;
  w_k_j(0) = kPiece->_curvature_binormal.dot(kPiece->_material_frame.col(2));
  w_k_j(1) = -kPiece->_curvature_binormal.dot(kPiece->_material_frame.col(1));
  double grad_E_theta = ((w_k_j.dot(JB*w_k_j)) + (2.0*TWIST_COEFF*(kPiece->_angle_twist - kPiece->_prev_piece->_angle_twist)))/(kPiece->_prev_piece->_rest_length + kPiece->_rest_length);
  grad -= (grad_E_theta*sum_writhe);


	grad = grad + Vector3d::UnitZ()*GRAV_COEFF;

#endif

}

void ThreadPiece::calc_del_kb_k(Matrix3d& del_kb_k, const ThreadPiece* other_piece, Matrix3d& edge_skew_prev, Matrix3d& edge_skew_next)
{
  if (other_piece == _prev_piece)
  {
    //del_i kb_i-1
    skew_symmetric_for_cross_fast(other_piece->_prev_piece->_edge, edge_skew_prev);
    del_kb_k = 2.0*edge_skew_prev - other_piece->_curvature_binormal*(other_piece->_prev_piece->_edge.transpose());
  } else if (other_piece == this) {
    //del_i kb_i
    skew_symmetric_for_cross_fast(other_piece->_prev_piece->_edge, edge_skew_prev);
    skew_symmetric_for_cross_fast(other_piece->_edge, edge_skew_next);
    del_kb_k = -2.0*edge_skew_prev + other_piece->_curvature_binormal*(other_piece->_prev_piece->_edge.transpose())
                -2.0*edge_skew_next - other_piece->_curvature_binormal*(other_piece->_edge.transpose());

  } else if (other_piece == _next_piece) {
    //del_i kb_i+1
    skew_symmetric_for_cross_fast(other_piece->_edge, edge_skew_next);
    del_kb_k = 2.0*edge_skew_next + other_piece->_curvature_binormal*(other_piece->_edge.transpose());
  } else {
    std::cerr << "gradient for this piece does not exist!" << std::endl;
    exit(0);
  }

  double denom = (other_piece->_prev_piece->_rest_length)*(other_piece->_rest_length) + other_piece->_prev_piece->_edge.dot(other_piece->_edge);
  del_kb_k /= denom;

}

void ThreadPiece::add_sum_writhe(ThreadPiece* other_piece, Vector3d& curr_sum)
{
  if (other_piece == _prev_piece)
  {
    curr_sum -= other_piece->_curvature_binormal/(2.0*(other_piece->_rest_length));
  } else if (other_piece == _next_piece) {
    curr_sum += other_piece->_curvature_binormal/(2.0*(other_piece->_rest_length));
  }
}


//not defined for first 2 or last 2 pieces
void ThreadPiece::gradient_vertex_numeric(Vector3d& grad)
{

  for (int grad_ind = 0; grad_ind < 3; grad_ind++)
  {
    offset_and_update_locally(grad_offsets[grad_ind]);
    grad[grad_ind] = _prev_piece->energy() + energy() + _next_piece->energy();
    offset_and_update_locally(-2.0*grad_offsets[grad_ind]);
    grad[grad_ind] -= _prev_piece->energy() + energy() + _next_piece->energy();
    offset_and_update_locally(grad_offsets[grad_ind]);
    grad[grad_ind] /= (2.0*grad_eps);
  }

}

//THIS MAKES THREAD DATA INCONSISTENT! Careful when using
void ThreadPiece::offset_and_update_locally(const Vector3d& offset)
{

  //offset this vertex (call it vertex i)
  _vertex += offset;

#ifdef ISOTROPIC
  updateFrames();
#else
  //calculate the locally changed edges and bishop frames (from i-1 to i+1)
	if (_prev_piece != NULL)
	{
		_prev_piece->update_edge();
		if (_prev_piece->_prev_piece != NULL) {
			_prev_piece->update_bishop_frame();
			_prev_piece->update_material_frame();
		}
	}

	if (_next_piece != NULL)
	{
		update_edge();
		if (_prev_piece != NULL)
		{
			update_bishop_frame();
		}
		if (_next_piece->_next_piece != NULL)
		{
			_next_piece->update_bishop_frame();
			//find how the change in bishop frame affects the angle change
			update_material_frame();
			double angle_twist_diff = _next_piece->twist_angle_error();
			// std::cout << "angle twist diff: " << angle_twist_diff << std::endl;
			_next_piece->_angle_twist += angle_twist_diff;
			//_next_piece->update_material_frame();
		}
	}
#endif
}

void ThreadPiece::offset_and_update(const Vector3d& offset)
{
  //offset this vertex
  _vertex += offset;
  updateFrames();
}


//not applicable for first 2 or last 2 pieces
void ThreadPiece::updateFrames()
{
  /*if (_prev_piece == NULL || _prev_piece->_prev_piece == NULL || _next_piece == NULL || _next_piece->_next_piece == NULL)
  {
    std::cerr << "shouldn't call updateFrames() on the first 2 or last 2 pieces!" << std::endl;
    exit(0);
  }*/

  _prev_piece->update_edge();
  update_edge();

  ThreadPiece* to_change_bishop = _prev_piece;
  while (to_change_bishop->_next_piece != NULL)
  {
    to_change_bishop->update_bishop_frame();
    to_change_bishop = to_change_bishop->_next_piece;
  }


#ifdef ISOTROPIC
  //angle of last piece is the only one that matters
  to_change_bishop = to_change_bishop->_prev_piece;
  to_change_bishop->_angle_twist += to_change_bishop->twist_angle_error();
#else
  //twist angle may have changed
  double angle_twist_diff = _next_piece->twist_angle_error();
  //_prev_piece->_angle_twist += angle_twist_diff/3.0;
  _prev_piece->update_material_frame();
  //_angle_twist += 2.0*angle_twist_diff/3.0;
  update_material_frame();

  ThreadPiece* to_change_twist = _next_piece;
  while (to_change_twist->_next_piece != NULL)
  {
    to_change_twist->_angle_twist += angle_twist_diff;
    to_change_twist = to_change_twist->_next_piece;
  }
#endif
}

//update frames for many vertex position changes
//assumes this is the first piece of thread
void ThreadPiece::updateFrames_all()
{
	update_edge();
	update_bishop_frame_firstPiece();
	ThreadPiece* curr_piece = _next_piece;

#ifdef ISOTROPIC
	for ( ; curr_piece->_next_piece != NULL; curr_piece = curr_piece->_next_piece)
	{
		curr_piece->update_edge();
		curr_piece->update_bishop_frame();
	}
  
  curr_piece = curr_piece->_prev_piece;
  curr_piece->_angle_twist += curr_piece->twist_angle_error();

#else
	update_material_frame();
	double twist_to_add = 0;
	for ( ; curr_piece->_next_piece != NULL; curr_piece = curr_piece->_next_piece)
	{
		curr_piece->update_edge();
		curr_piece->update_bishop_frame();
		curr_piece->_angle_twist += twist_to_add;
		double this_err = curr_piece->twist_angle_error();
		twist_to_add += this_err;
		curr_piece->_angle_twist += this_err;
		curr_piece->update_material_frame();
	}
#endif
}

//should not be used on first 1 or last 2 pieces
void ThreadPiece::updateFrames_twistOnly()
{
  update_material_frame();
}

void ThreadPiece::updateFrames_firstpiece()
{
  updateFrames_all();
  /*
  //1st edge always has material frame = bishop frame, with twist angle zero
  _next_piece->update_bishop_frame();

  //for 2nd piece, look ahead to see what the angle error is, and add that twist angle
  double angle_twist_diff = _next_piece->_next_piece->twist_angle_error();
  _next_piece->_angle_twist += angle_twist_diff;
  _next_piece->update_material_frame();
  //update everything else
  _next_piece->_next_piece->updateFrames();
  */
}

void ThreadPiece::updateFrames_lastpiece()
{
  _prev_piece->_material_frame = _material_frame;
  _prev_piece->_prev_piece->update_edge();
  _prev_piece->update_edge();

  _prev_piece->_prev_piece->update_bishop_frame();
  _prev_piece->update_bishop_frame();

  double err = _prev_piece->twist_angle_error();
  //_prev_piece->_prev_piece->_angle_twist += err/2.0;
  _prev_piece->_angle_twist += err;
  //std::cout << "err of " << err << "makes total angle of " << _prev_piece->_angle_twist*360.0/(2.0*M_PI) << std::endl;

#ifndef ISOTROPIC
  _prev_piece->_prev_piece->update_material_frame();
#endif

  _prev_piece->update_material_frame();

}




void ThreadPiece::update_bishop_frame()
{
	calculateBinormal();
  //now rotate frame
  double curvature_binormal_norm = _curvature_binormal.norm();
  Vector3d toRotAround = _curvature_binormal/curvature_binormal_norm;

  //this is last material frame rotated with curvature
  if (isnan(toRotAround(0)))
  {
    _bishop_frame = _prev_piece->_bishop_frame;
  } else {
    
    double for_ang = (_prev_piece->_edge.dot(_edge))/(_prev_piece->_edge_norm*_edge_norm);
    for_ang = max( min ( for_ang, 1.0), -1.0);
    double ang_to_rot_old = acos(for_ang);
    
    
    //double ang_to_rot = atan2(curvature_binormal_norm/2.0)*2.0;
    
    set_rotation(ang_to_rot_old, toRotAround);
    _bishop_frame = rot*_prev_piece->_bishop_frame;

    //this never seemed to happen, so comment for efficiency?
    double old_err = (_bishop_frame.col(0) - _edge.normalized()).norm();
    if ( old_err > 1e-5)
    {
      //std::cout << "old err: " << old_err;
      set_rotation(-ang_to_rot_old, toRotAround);
      _bishop_frame = rot*_prev_piece->_bishop_frame;

      double new_err = (_bishop_frame.col(0) - _edge.normalized()).norm();
      //std::cout << "new err: " << new_err << std::endl;
      //
      //in case it actually got worse, switch back
      if (new_err > old_err)
      {
        set_rotation(ang_to_rot_old, toRotAround);
        _bishop_frame = rot*_prev_piece->_bishop_frame;
      }
    }
    

   // _bishop_frame = Eigen::AngleAxisd(ang_to_rot, toRotAround)*_prev_piece->_bishop_frame;
  }

}



void ThreadPiece::update_bishop_frame_firstPiece()
{
	Vector3d new_direction = _edge.normalized();
	if ( (new_direction-_bishop_frame.col(0)).norm() >= 1e-4)
	{
		Vector3d to_be_perp = _bishop_frame.col(1);

		make_vectors_perpendicular(new_direction, to_be_perp);

		_bishop_frame.col(0) = new_direction;
		_bishop_frame.col(1) = to_be_perp;
		_bishop_frame.col(2) = (_bishop_frame.col(0).cross(_bishop_frame.col(1))).normalized();
	}

}


void ThreadPiece::update_bishop_frame_lastPiece()
{
	_bishop_frame = _prev_piece->_bishop_frame;
}





void ThreadPiece::set_rotation(const double& angle, const Vector3d& axis) {
    //rot.setZero();
    double x = axis(0); double y = axis(1); double z = axis(2);
    double cost = cos(angle);
    double m1cost = 1.0-cost;
    double sint = sin(angle);
    rot <<
        cost + x*x*m1cost, x*y*m1cost - z*sint, x*z*m1cost + y*sint,
        y*x*m1cost + z*sint, cost + y*y*m1cost, y*z*m1cost - x*sint,
        z*x*m1cost - y*sint, z*y*m1cost + x*sint, cost + z*z*m1cost;
}

void ThreadPiece::update_material_frame()
{
  //if (_next_piece==NULL || _next_piece->_next_piece == NULL || _prev_piece==NULL)
  //  return;
	if (_angle_twist == 0.0) {
		_material_frame = _bishop_frame;
	}	else {
    set_rotation(_angle_twist, _edge/_edge_norm);
		//rot = Eigen::AngleAxisd(_angle_twist, _edge/_edge_norm);
    _material_frame = rot*_bishop_frame;
	}
}

/*
double ThreadPiece::calculate_holonomy()
{
	if (_prev_piece == NULL || _next_piece == NULL)
		return 0.0;


	Matrix3d curr_rot = _bishop_frame;
	Vector3d binormal_back;
	Vector3d edge_back = _prev_piece->_vertex - _next_piece->_vertex;

	binormal_back = _edge.cross(edge_back);
	binormal_back.normalize();
	//if (!isnan(binormal_back(0)))
	//{
		double ang_back = (_edge.dot(edge_back))/(_edge_norm*edge_back.norm());
		ang_back = max( min (ang_back, 1.0), -1.0);
		double ang_to_rot_back = acos(ang_back);
		curr_rot = Eigen::AngleAxisd(ang_to_rot_back, binormal_back)*curr_rot;
	//}

	//now rot in direction of edge

	Vector3d binormal_align;
	binormal_align = edge_back.cross(_prev_piece->_edge);
	binormal_align.normalize();
	//if (!isnan(binormal_align(0)))
	//{
		double ang_align = edge_back.dot(_prev_piece->_edge)/(edge_back.norm()*_prev_piece->_edge_norm);
		ang_align = max( min (ang_align, 1.0), -1.0);
		double ang_to_rot_align = acos(ang_align);
		curr_rot = Eigen::AngleAxisd(ang_to_rot_align, binormal_align)*curr_rot;
	//}

	std::cout << "mismatch: " << angle_mismatch(curr_rot, _prev_piece->_bishop_frame) << std::endl;
	std::cout << "angs: " << ang_to_rot_back << "  " << ang_to_rot_align << std::endl;

	std::cout << "bishop:\n" << _prev_piece->_bishop_frame << std::endl;
	std::cout << "curr rot:\n" << curr_rot << std::endl;
	std::cout << "edge prev: " << _prev_piece->_edge.normalized().transpose() << std::endl;
	std::cout << "edge: " << _edge.normalized().transpose() << std::endl;
	std::cout << "edge back: " << edge_back.normalized().transpose() << std::endl;
	std::cout << "cross: " << binormal_back.transpose() << std::endl;
	std::cout << "cross2: " << binormal_align.transpose() << std::endl;

	return angle_mismatch(curr_rot, _prev_piece->_bishop_frame);

}

*/

void ThreadPiece::update_edge()
{
  if (_next_piece == 0)
     return;
  _edge = _next_piece->_vertex - _vertex;
  _edge_norm = _edge.norm();
}


//assumes the angles are correct, and calculates the material frames
void ThreadPiece::initializeFrames()
{
  if (_next_piece == NULL)
	{
//		update_bishop_frame_lastPiece();
//		update_material_frame();
    return;
	}

  update_edge();

  //now update bishop and material frames
  if (_prev_piece != NULL)
  {
    update_bishop_frame();
    update_material_frame();
  } else {
		update_bishop_frame_firstPiece();
    update_material_frame();
	}

  _next_piece->initializeFrames();
}

bool ThreadPiece::is_material_frame_consistent()
{
  double norm_direction = (_material_frame.col(0) - _bishop_frame.col(0)).norm();
  double angle_err = abs(twist_angle_error());

  return (norm_direction < 1e-10 & angle_err < 1e-10);
}


double ThreadPiece::twist_angle_error()
{
  double angle_err_frames = angle_mismatch(_material_frame, _bishop_frame);
  /*
  double angle = angle_err_frames - _angle_twist;
  while (angle < -M_PI)
    angle += 2.0*M_PI;
  while (angle > M_PI)
    angle -= 2.0*M_PI;
  return angle;
  */

  return atan2( sin(angle_err_frames - _angle_twist), cos(angle_err_frames - _angle_twist));
}


/*void ThreadPiece::calculateBinormal(const double rest_length_prev, const Vector3d& edge_prev, 
																		const double rest_length_after, const Vector3d& edge_after, Vector3d& binormal)
{
	if (_prev_piece == NULL) 
		cout << "Internal error: ThreadPiece::calculateBinormal() : _prev_piece is NULL." << endl;
  binormal = 2.0*edge_prev.cross(edge_after);
  binormal /= ((rest_length_prev * rest_length_after) + edge_prev.dot(edge_after));
}*/

void ThreadPiece::calculateBinormal()
{
	if (_prev_piece == NULL) 
		cout << "Internal error: ThreadPiece::calculateBinormal() : _prev_piece is NULL." << endl;
  _curvature_binormal = 2.0*_prev_piece->_edge.cross(_edge);
  _curvature_binormal /= (_prev_piece->_edge_norm*_edge_norm + _prev_piece->_edge.dot(_edge));
}

/*void ThreadPiece::calculateBinormal_withLength(const Vector3d& edge_prev, const Vector3d& edge_after, Vector3d& binormal)
{
  binormal = 2.0*edge_prev.cross(edge_after);
  binormal /= (edge_prev.norm()*edge_after.norm() + edge_prev.dot(edge_after));
}

void ThreadPiece::calculateBinormal_withLength()
{
  _curvature_binormal = 2.0*_prev_piece->_edge.cross(_edge);
  _curvature_binormal /= (_prev_piece->_edge_norm*_edge_norm + _prev_piece->_edge.dot(_edge));
}*/

//variable-length thread_pieces
// Splits the edge between this->vertex and this->_next_piece->_vertex, i.e. splits this into this and new_piece.
// this->_vertex shouldn't be any of the first or last vertices of the thread.
void ThreadPiece::splitPiece(ThreadPiece* new_piece)
{	
	if (_prev_piece==NULL)
		cout << "Internal error: ThreadPiece::splitPiece: this->_vertex cannot be the first vertex." << endl;
	if (_next_piece==NULL || _next_piece->_next_piece==NULL)
		cout << "Internal error: ThreadPiece::splitPiece: this->_vertex cannot be the last or second to last vertex." << endl;
	new_piece->_vertex = (_vertex + _next_piece->_vertex)/2.0;
  new_piece->_angle_twist = _angle_twist/2.0;
  //_edge, _edge_norm and _curvature_binormal are updated later
	//new_piece->_bishop_frame = _bishop_frame;
	//new_piece->_material_frame = _material_frame;
	new_piece->_rest_length = _rest_length = _rest_length/2.0;
	
	fixPointersSplit(new_piece);
	
	update_edge();
	calculateBinormal();
	//update_bishop_frame();
	//update_material_frame();
	new_piece->update_edge();
	new_piece->calculateBinormal();
	new_piece->update_bishop_frame();
	new_piece->update_material_frame();
}

// Merges the edges adjacent to this->_vertex, i.e. merges this->_prev_piece with this and puts it into this.
// this->_vertex shouldn't be any of the first two or last two vertices of the thread.
void ThreadPiece::mergePiece()
{
	if (_prev_piece==NULL || _prev_piece->_prev_piece == NULL)
		cout << "Internal error: ThreadPiece::mergePiece: this->_vertex cannot be the first or second vertex." << endl;
	else if (_next_piece==NULL || _next_piece->_next_piece == NULL)
		cout << "Internal error: ThreadPiece::mergePiece: this->_vertex cannot be the last or second to last vertex." << endl;
	
	_vertex = _prev_piece->_vertex;
  _angle_twist = _prev_piece->_angle_twist;
	//_edge, _edge_norm and _curvature_binormal are updated later
	//intermediate_rotation(_bishop_frame, _prev_piece->_bishop_frame, _bishop_frame);
	//intermediate_rotation(_material_frame, _prev_piece->_material_frame, _material_frame);
	_rest_length = _prev_piece->_rest_length + _rest_length;
	
	fixPointersMerge();
	
	update_edge();
	calculateBinormal();
	update_bishop_frame();
	update_material_frame();
}

void ThreadPiece::fixPointersSplit(ThreadPiece* new_piece)
{
	this->_next_piece->_prev_piece = new_piece;
	new_piece->_prev_piece = this;
	new_piece->_next_piece = _next_piece;
	this->_next_piece = new_piece;
	new_piece->_my_thread = _my_thread;
}

void ThreadPiece::fixPointersMerge()
{
	this->_prev_piece->_prev_piece->_next_piece = this;
	this->_prev_piece = _prev_piece->_prev_piece;
}

//collision
bool ThreadPiece::isPieceIndConsistent()
{
	int piece_ind;
  for (piece_ind=0; piece_ind<_my_thread->_thread_pieces.size() && _my_thread->_thread_pieces[piece_ind]!=this; piece_ind++) {}
	assert(piece_ind != _my_thread->_thread_pieces.size()); //this piece is not in _my_thread->_thread_pieces.
}

void ThreadPiece::updateCollisionObjectTransform()
{
	assert(_next_piece != NULL);
	_col_obj->getWorldTransform().setOrigin(tobtVector3((_vertex + _next_piece->_vertex)/2.0)); //(_edge_norm/2.0) * _material_frame.col(0) + _vertex
	_col_obj->getWorldTransform().setBasis(tobtMatrix3x3(_material_frame));
}

//ThreadPiece& ThreadPiece::operator=(const ThreadPiece& rhs)
//{
//  _vertex = rhs._vertex;
//  _angle_twist = rhs._angle_twist;
//	_edge = rhs._edge;
//	_edge_norm = rhs._edge_norm;
//	_curvature_binormal = rhs._curvature_binormal;
//	_bishop_frame = rhs._bishop_frame;
//	_material_frame = rhs._material_frame;
//	_rest_length = rhs._rest_length;

//  _prev_piece = rhs._prev_piece;
//  _next_piece = rhs._next_piece;
//  _my_thread = rhs._my_thread;

//	_col_obj.setCollisionShape(dynamic_cast<btCapsuleShapeX*>((btCollisionShape*) rhs._col_obj.getCollisionShape()));

//	return *this;

//}

void ThreadPiece::copyData(const ThreadPiece& rhs)
{
  _vertex = rhs._vertex;
  _angle_twist = rhs._angle_twist;
	_edge = rhs._edge;
	_edge_norm = rhs._edge_norm;
	_curvature_binormal = rhs._curvature_binormal;
	_bishop_frame = rhs._bishop_frame;
	_material_frame = rhs._material_frame;
  _rest_length = rhs._rest_length;

	_col_obj = rhs._col_obj;
}

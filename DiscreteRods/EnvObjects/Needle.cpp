#include "Needle.h"
#include "../ThreadConstrained.h"

Needle::Needle(const Vector3d& pos, const Matrix3d& rot, double degrees, double r, float c0, float c1, float c2, World* w, ThreadConstrained* t, int constrained_vertex_num)
	: EnvObject(c0, c1, c2, NEEDLE)
	, angle(degrees)
	, radius(r)
	, thread(t)
	, constraint(constrained_vertex_num)
	, world(w)
	, position_offset(0.0, 0.0, 0.0)
	, rotation_offset(Matrix3d::Identity())
{
	assert(((thread == NULL) && (constrained_vertex_num == -1)) || ((thread != NULL) && (constrained_vertex_num != -1)));
	updateConstraintIndex();
	
	assert(degrees > MIN_ANGLE);
	assert(r > MIN_RADIUS);

	double arc_length = radius * angle * M_PI/180.0;
	int pieces = ceil(arc_length/2.0);
  for (int piece=0; piece<pieces; piece++) {
 		Intersection_Object* obj = new Intersection_Object();
 		obj->_radius = radius/8.0;
  	i_objs.push_back(obj);
	}	

	if (thread != NULL) {
		Matrix3d new_rot = AngleAxisd(-M_PI/2.0, rot.col(0)) * (AngleAxisd((angle) * M_PI/180.0, rot.col(1)) * rot);
		setTransform(pos - radius * rot.col(1), new_rot);
	} else {
		setTransform(pos, rot);
	}
}

//Needle::Needle(const Vector3d& pos, const Vector3d& start_pos, const Vector3d& end_pos, float c0, float c1, float c2, World* w, ThreadConstrained* t, int constrained_vertex_num)
//	: EnvObject(c0, c1, c2, NEEDLE)
//	, thread(t)
//	, constraint(constrained_vertex_num)
//	, world(w)
//	, position_offset(0.0, 0.0, 0.0)
//	, rotation_offset(Matrix3d::Identity())
//{
//	assert(((thread == NULL) && (constrained_vertex_num == -1)) || ((thread != NULL) && (constrained_vertex_num != -1)));
//	updateConstraintIndex();
//	
//	double center_start_length = (start_pos - pos).norm();
//	double center_end_length = (end_pos - pos).norm();
//	assert(center_start_length > MIN_RADIUS);
//	assert((center_start_length - center_end_length) < 1e-10);
//	radius = center_start_length;
//	rotation.col(2) = (start_pos - pos).normalized();
//	rotation.col(1) = ((end_pos - pos).normalized()).cross(rotation.col(2));
//	rotation.col(0) = rotation.col(1).cross(rotation.col(2));
//	angle = (180.0/M_PI) * acos((start_pos - pos).dot(end_pos - pos) / (center_start_length * center_end_length));
//	assert(angle > MIN_ANGLE);
//	
//	double arc_length = radius * angle * M_PI/180.0;
//	int pieces = ceil(arc_length/2.0);
//  for (int piece=0; piece<pieces; piece++) {
// 		Intersection_Object* obj = new Intersection_Object();
// 		obj->_radius = radius/8.0;
//  	i_objs.push_back(obj);
//	}	
//	setTransform(pos, rotation);
//}

//Needle::Needle(const Vector3d& pos, const Vector3d& center_start, double degrees, const Vector3d& axis, float c0, float c1, float c2, World* w, ThreadConstrained* t, int constrained_vertex_num)
//	: EnvObject(pos, Matrix3d::Identity(), c0, c1, c2, NEEDLE)
//	, angle(degrees)
//	, radius(center_start.norm())
//	, thread(t)
//	, constraint(constrained_vertex_num)
//	, world(w)
//	, position_offset(0.0, 0.0, 0.0)
//	, rotation_offset(Matrix3d::Identity())
//{
//	assert(((thread == NULL) && (constrained_vertex_num == -1)) || ((thread != NULL) && (constrained_vertex_num != -1)));
//	updateConstraintIndex();
//	
//	assert(degrees > MIN_ANGLE);
//	assert(radius > MIN_RADIUS);
//	rotation.col(2) = center_start.normalized();
//	rotation.col(1) = axis.normalized();
//	rotation.col(0) = rotation.col(1).cross(rotation.col(2));
//	
//	double arc_length = radius * angle * M_PI/180.0;
//	int pieces = ceil(arc_length/2.0);
//  for (int piece=0; piece<pieces; piece++) {
// 		Intersection_Object* obj = new Intersection_Object();
// 		obj->_radius = radius/8.0;
//  	i_objs.push_back(obj);
//	}	
//	setTransform(pos, rotation);
//}

Needle::Needle(const Needle& rhs, World* w)
	: EnvObject(rhs.color0, rhs.color1, rhs.color2, rhs.type)
	, angle(rhs.angle)
	, radius(rhs.radius)
	, thread(rhs.thread)
	, constraint(rhs.constraint)
	, world(w)
	, position_offset(0.0, 0.0, 0.0)
	, rotation_offset(Matrix3d::Identity())
{
	assert(type == NEEDLE);
  
  if (thread == NULL) {
		assert(constraint == -1);
		constraint_ind = -1;
	} else {
		assert(constraint != -1);
		thread = world->objectAtIndex<ThreadConstrained>(rhs.world->objectIndex<ThreadConstrained>(rhs.thread));
		vector<int> constrained_vertices_nums;
		thread->getConstrainedVerticesNums(constrained_vertices_nums);
		constraint_ind = find(constrained_vertices_nums, constraint);
		assert(constraint_ind != -1);
	}
  
	double arc_length = radius * angle * M_PI/180.0;
	int pieces = ceil(arc_length/2.0);
  for (int piece=0; piece<pieces; piece++) {
 		Intersection_Object* obj = new Intersection_Object();
 		obj->_radius = radius/8.0;
  	i_objs.push_back(obj);
	}	
	setTransform(rhs.position, rhs.rotation);
}
		
Needle::~Needle()
{}

void Needle::writeToFile(ofstream& file)
{
	assert(type == NEEDLE);
	file << type << " ";
	for (int i=0; i<3; i++)
		file << position(i) << " ";
	for (int r=0; r < 3; r++)
    for (int c=0; c < 3; c++)
      file << rotation(r,c) << " ";
  file << angle << " " << radius << " " << color0 << " " << color1 << " " << color2 << " " << constraint << " " << world->objectIndex<ThreadConstrained>(thread) << " ";
  file << "\n";
}

Needle::Needle(ifstream& file, World* w)
	: thread(NULL)
	, world(w)
	, position_offset(0.0, 0.0, 0.0)
	, rotation_offset(Matrix3d::Identity())
{
  type = NEEDLE;
  
	for (int i=0; i<3; i++)
		file >> position(i);
	for (int r=0; r < 3; r++)
    for (int c=0; c < 3; c++)
      file >> rotation(r,c);

  file >> angle >> radius >> color0 >> color1 >> color2;
  
  int world_thread_ind;
  file >> constraint >> world_thread_ind;
 	thread = world->objectAtIndex<ThreadConstrained>(world_thread_ind);
 	updateConstraintIndex();
	assert(((thread == NULL) && (constraint == -1) && (constraint_ind == -1)) || 
				 ((thread != NULL) && (constraint != -1) && (constraint_ind != -1)));
  
  double arc_length = radius * angle * M_PI/180.0;
	int pieces = ceil(arc_length/2.0);
  for (int piece=0; piece<pieces; piece++) {
 		Intersection_Object* obj = new Intersection_Object();
 		obj->_radius = radius/8.0;
  	i_objs.push_back(obj);
	}
	setTransform(position, rotation);
}

//pos and rot correspond to the needle transform
void Needle::setTransform(const Vector3d& pos, const Matrix3d& rot)
{
	position = pos;
	rotation = rot;
		
	double angle_per_link = angle/i_objs.size();
	
	i_objs[0]->_start_pos = position + radius * rotation.col(2);
	i_objs[0]->_end_pos = position + radius * (rotation * AngleAxisd(-angle_per_link * M_PI/180.0, Vector3d::UnitY())).col(2);
	for (int piece=1; piece<i_objs.size(); piece++) {
 		i_objs[piece]->_start_pos = i_objs[piece-1]->_end_pos;
		i_objs[piece]->_end_pos = position + radius * (rotation * AngleAxisd(-angle_per_link * (piece+1) * M_PI/180.0, Vector3d::UnitY())).col(2);
	}
}

//ROTATION RELATIONS BETWEEN NEEDLE AND OTHER OBJECTS
//NEEDLE-END_EFFECTOR
//needle_rotation = ee_rot * rotation_offset
//needle_position = ee_pos + ee_rot*rotation_offset*position_offset
//ee_rot = needle_rotation * rotation_offset.transpose()
//ee_pos = needle_position - ee_rot*rotation_offset*position_offset

//NEEDLE-THREAD_CONSTRAINED
//thread_rot = AngleAxisd(-angle * M_PI/180.0, needle_rotation.col(1)) * needle_rotation = getEndRotation()
//thread_pos = needle_position + radius * (AngleAxisd(-angle * M_PI/180.0, needle_rotation.col(1)) * needle_rotation).col(2) = getEndPosition()
//needle_rotation = AngleAxisd(angle * M_PI/180.0, thread_rot.col(1)) * thread_rot
//needle_position = thread_pos - radius * (AngleAxisd(-angle * M_PI/180.0, needle_rotation.col(1)) * needle_rotation).col(2)

//FACT
//needle_rotation.col(1) = thread_rot.col(1)

void Needle::setTransformFromEndEffector(const Vector3d& ee_pos, const Matrix3d& ee_rot)
{
	setTransform(ee_pos + ee_rot*rotation_offset*position_offset, ee_rot * rotation_offset);

	if(isThreadAttached()) {
		thread->updateConstrainedTransform(constraint_ind, getEndPosition(), getEndRotation());
		//thread->minimize_energy();
	}
}

//call this when the end effector just attaches to the needle, so that the needle know the relative position and orientation at the time of the attachment.
void Needle::setTransformOffsetFromEndEffector(const Vector3d& ee_pos, const Matrix3d& ee_rot)
{
	rotation_offset = ee_rot.transpose() * rotation;
	position_offset = rotation_offset.transpose()*ee_rot.transpose()*(position - ee_pos);
}

void Needle::getEndEffectorTransform(Vector3d& ee_pos, Matrix3d& ee_rot)
{
	ee_rot = rotation * rotation_offset.transpose();
	ee_rot.col(1) = ee_rot.col(2).cross(ee_rot.col(0));
	ee_rot.col(2) = ee_rot.col(0).cross(ee_rot.col(1));
	ee_rot.col(0).normalize();
	ee_rot.col(1).normalize();
	ee_rot.col(2).normalize();
	
	ee_pos = position - ee_rot*rotation_offset*position_offset;
}

void Needle::setTransformFromEndEffectorBoxConstrained(const Vector3d& new_ee_pos, const Matrix3d& new_ee_rot)
{
	Vector3d old_ee_pos;
	Matrix3d old_ee_rot;
	getEndEffectorTransform(old_ee_pos, old_ee_rot);

	Vector3d old_proj = (old_ee_pos - position);
	old_proj.normalize();
	Vector3d proj = (new_ee_pos-position).dot(rotation.col(0))*rotation.col(0) + (new_ee_pos-position).dot(rotation.col(2))*rotation.col(2);
	proj.normalize();

	glColor3f(0.8,0.8,0);
	drawArrow(position, 20*proj);
	glColor3f(0.8,0,0);
	drawArrow(position, 20*old_proj);

	Vector3d normal = proj.cross(old_proj);
	normal.normalize();

	double angle = angle_between(proj, old_proj)*180.0/M_PI;
	if (angle > 90.0) {
		angle = 0.0;
		//TODO return;
	} else {
		double sign = ((normal - rotation.col(1)).squaredNorm() < 1e-5) ? -1 : 1;
		angle = sign*min(angle_between(proj, old_proj)*180.0/M_PI, 5.0);
	}

	if (!isnan(angle) && abs(angle)>1e-5 && angle>0.0) { //TODO for now, only allows forward movement of needle through box
		rotateAboutAxis(angle);
		if(isThreadAttached()) {
			thread->updateConstrainedTransform(constraint_ind, getEndPosition(), getEndRotation());
		}
	}	
}

void Needle::updateTransformFromAttachment()
{
	if (isThreadAttached()) {
		const Matrix3d thread_rot = thread->rotationAtConstraint(constraint_ind);
		const Vector3d thread_pos = thread->positionAtConstraint(constraint_ind);
		const Matrix3d needle_rot = AngleAxisd(angle * M_PI/180.0, thread_rot.col(1)) * thread_rot;
		const Vector3d needle_pos = thread_pos - radius * (AngleAxisd(-angle * M_PI/180.0, needle_rot.col(1)) * needle_rot).col(2);
		setTransform(needle_pos, needle_rot);
	}
}

Vector3d Needle::getStartPosition()
{
	return position + radius * rotation.col(2);
}

Vector3d Needle::getEndPosition()
{
	return position + radius * (AngleAxisd(-angle * M_PI/180.0, rotation.col(1)) * rotation).col(2);
}

Matrix3d Needle::getStartRotation()
{
	return rotation;
}

Matrix3d Needle::getEndRotation()
{
	return AngleAxisd(-angle * M_PI/180.0, rotation.col(1)) * rotation;
}

double Needle::getAngle()
{
	return angle;
}

double Needle::getCurvatureRadius()
{
	return radius;
}

double Needle::getThicknessRadius()
{
	return radius/8.0;
}

Vector3d Needle::getAxis()
{
	return rotation.col(1);
}

void Needle::rotateAboutAxis(double degrees)
{
	setTransform(position, AngleAxisd(degrees * M_PI/180.0, rotation.col(1)) * rotation);
	if(isThreadAttached()) {
		thread->updateConstrainedTransform(constraint_ind, getEndPosition(), getEndRotation());
	}
}

Vector3d Needle::nearestPosition(const Vector3d& pos)
{
//	Matrix<double, 3, 2> A;
//	A.col(0) = rotation.col(2);
//	A.col(1) = (rotation * AngleAxisd(-angle * M_PI/180.0, rotation.col(1))).col(2);
//	Vector3d B = pos - position;
//	Vector3d proj_B = A*(A.transpose()*A).inverse()*A.transpose()*B;
//	double middle = (pos - position + radius * proj_B.normalized()).squaredNorm();
//	double start = (pos - getStartPosition()).squaredNorm();
//	double end = (pos - getEndPosition()).squaredNorm();
//	if ((middle < start) && (middle < end)) {
//		return position + radius * proj_B.normalized();
//	} else {
//		if (start < end)
//			return getStartPosition();
//		else
//			return getEndPosition();
//	}
	
	double angle_per_link = angle/i_objs.size();

	Vector3d min_pos = position + radius * rotation.col(2);
	double min_dist = (min_pos - pos).squaredNorm();
	for (int piece=1; piece<i_objs.size()+1; piece++) {
 		Vector3d candidate_pos = position + radius * (rotation * AngleAxisd(-angle_per_link * (piece) * M_PI/180.0, Vector3d::UnitY())).col(2);
 		double candidate_dist = (candidate_pos - pos).squaredNorm();
 		if (candidate_dist < min_dist) {
 			min_pos = candidate_pos;
 			min_dist = candidate_dist;
 		}
	}
	return min_pos;
}

void Needle::draw()
{
	glColor3f(color0, color1, color2);
 	int obj_ind;
  for (obj_ind = 0; obj_ind<i_objs.size(); obj_ind++) {
 		drawCylinder(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_end_pos, i_objs[obj_ind]->_radius);
		drawSphere(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_radius);
	}	
	drawSphere(i_objs[obj_ind-1]->_end_pos, i_objs[obj_ind-1]->_radius);
	glColor3f(1,0,0);
	drawSphere(i_objs[0]->_start_pos, i_objs[0]->_radius);
}

void Needle::drawDebug()
{
//	glColor3f(0,1,0);
//	drawSphere(getStartPosition(), 1);		
//	drawAxes(getStartPosition(), getStartRotation());
//	drawAxes(getEndPosition(), getEndRotation());

//	Vector3d old_ee_pos;
//	Matrix3d old_ee_rot;
//	getEndEffectorTransform(old_ee_pos, old_ee_rot);

//	vector<Cursor*> cursors;
//	world->getObjects<Cursor>(cursors);
//	Vector3d new_ee_pos = cursors[1]->getPosition();
//	Matrix3d new_ee_rot = cursors[1]->getRotation();

//	Vector3d old_proj = (old_ee_pos - position);
//	old_proj.normalize();
//	Vector3d proj = (new_ee_pos-position).dot(rotation.col(0))*rotation.col(0) + (new_ee_pos-position).dot(rotation.col(2))*rotation.col(2);
//	proj.normalize();
//	
//	glColor3f(0.8,0.8,0);
//	drawArrow(position, 20*proj);
//	glColor3f(0.8,0,0);
//	drawArrow(position, 20*old_proj);
//	
//	Vector3d normal = proj.cross(old_proj);
//	normal.normalize();

//	double angle = angle_between(proj, old_proj)*180.0/M_PI;
//	if (angle > 90.0) {
//		angle = 0.0;
//		//TODO return;
//	} else {
//		double sign = ((normal - rotation.col(1)).squaredNorm() < 1e-5) ? -1 : 1;
//		angle = sign*min(angle_between(proj, old_proj)*180.0/M_PI, 5.0);
//	}
//	
//	if (!isnan(angle) && angle>1e-5)
//		rotateAboutAxis(angle);
}

void Needle::updateConstraint()
{
	if (constraint_ind != -1) {
		assert(thread != NULL);
		vector<int> constrained_vertices_nums;
		thread->getConstrainedVerticesNums(constrained_vertices_nums);
		constraint = constrained_vertices_nums[constraint_ind];
	} else {
		assert(thread == NULL);
		constraint = -1;
	}
}

void Needle::updateConstraintIndex()
{
	if (constraint != -1) {
		assert(thread != NULL);
		vector<int> constrained_vertices_nums;
		thread->getConstrainedVerticesNums(constrained_vertices_nums);
		constraint_ind = find(constrained_vertices_nums, constraint);
		assert(constraint_ind != -1); //constraint is supposed to be in constrained_vertices_nums but it isn't
	} else {
		assert(thread == NULL);
		constraint_ind = -1;
	}
}

void Needle::backup()
{
	backup_position = position;
	backup_rotation = rotation;
	backup_position_offset = position_offset;
	backup_rotation_offset = rotation_offset;
	backup_constraint = constraint;
	backup_thread_ind = world->objectIndex<ThreadConstrained>(thread);
}

// caller is responsible for having backedup before restoring
void Needle::restore()
{
	position_offset = backup_position_offset;
	rotation_offset = backup_rotation_offset;
	setTransform(backup_position, backup_rotation);
	constraint = backup_constraint;
	thread = world->objectAtIndex<ThreadConstrained>(backup_thread_ind);
	updateConstraintIndex();
}

bool Needle::boxCollision(const Vector3d& b_center, const Vector3d& b_half_length)
{
	Vector3d direction;
	for (int piece=0; piece<i_objs.size(); piece++) {
		if (capsuleBoxDistance(i_objs[piece]->_start_pos, i_objs[piece]->_end_pos , i_objs[piece]->_radius, b_center, b_half_length, direction) < 0.0)
 		return true;	
	}
	return false;
}

bool Needle::capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections)
{
	return false;
}

double Needle::capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius)
{
	return 0.0;
}

void Needle::capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient)
{
	return;
}

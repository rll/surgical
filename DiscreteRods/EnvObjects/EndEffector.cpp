#include "EndEffector.h"
#include "World.h"
#include "../ThreadConstrained.h"

EndEffector::EndEffector(const Vector3d& pos, const Matrix3d& rot, World* w, ThreadConstrained* t, int constrained_vertex_num)
	: EnvObject(default_color0, default_color1, default_color2, END_EFFECTOR)
	, thread(t)
	, constraint(constrained_vertex_num)
	, needle(NULL)
	, world(w)
	, open(false)
{
	assert(((thread == NULL) && (constrained_vertex_num == -1)) || ((thread != NULL) && (constrained_vertex_num != -1)));
	/*if (thread == NULL) {
		constraint_ind = -1;
	} else {
		vector<int> constrained_vertices_nums;
		thread->getConstrainedVerticesNums(constrained_vertices_nums);
		constraint_ind = find(constrained_vertices_nums, constraint);
		assert(constraint_ind != -1);
	}*/
	updateConstraintIndex();
  
	btCapsuleShapeX* capsule_short_handle = new btCapsuleShapeX(btScalar(short_handle_r+REPULSION_DIST), btScalar(short_handle_h));
	capsule_short_handle->setMargin(0.f);
	btCollisionObject* short_handle = new btCollisionObject();
	short_handle->setCollisionShape(capsule_short_handle);
	short_handle->setUserPointer(this);
	col_objs.push_back(short_handle);
	
	btCapsuleShapeX* capsule_handle = new btCapsuleShapeX(btScalar(handle_r+4.0*REPULSION_DIST), btScalar(handle_h));
	capsule_handle->setMargin(0.f);
	btCollisionObject* handle = new btCollisionObject();
	handle->setCollisionShape(capsule_handle);
	handle->setUserPointer(this);
	if (world->collision_world != NULL)
		world->collision_world->addCollisionObject(handle);
	col_objs.push_back(handle);

	for (int piece=0; piece<pieces; piece++) {
		btCapsuleShapeX* capsule_tip_piece = new btCapsuleShapeX(btScalar(0.9+(double(piece))*(handle_r-0.9)/(pieces-1.0)+REPULSION_DIST), btScalar(h));
		capsule_tip_piece->setMargin(0.f);
		btCollisionObject* tip_piece = new btCollisionObject();
		tip_piece->setCollisionShape(capsule_tip_piece);
		tip_piece->setUserPointer(this);
		if (world->collision_world != NULL)
			world->collision_world->addCollisionObject(tip_piece);
		col_objs.push_back(tip_piece);
	}
	
	for (int piece=0; piece<pieces; piece++) {
		btCapsuleShapeX* capsule_tip_piece = new btCapsuleShapeX(btScalar(0.9+(double(piece))*(handle_r-0.9)/(pieces-1.0)+REPULSION_DIST), btScalar(h));
		capsule_tip_piece->setMargin(0.f);
		btCollisionObject* tip_piece = new btCollisionObject();
		tip_piece->setCollisionShape(capsule_tip_piece);
		tip_piece->setUserPointer(this);
		if (world->collision_world != NULL)
			world->collision_world->addCollisionObject(tip_piece);
		col_objs.push_back(tip_piece);
	}
	
	setTransform(pos, rot);
}

/*EndEffector::EndEffector(const EndEffector& rhs, ThreadConstrained* t, int constrained_vertex_num)
	: EnvObject(rhs.color0, rhs.color1, rhs.color2, rhs.type)
	, thread(t)
	, constraint(constrained_vertex_num)
	, backup_thread_ind(rhs.backup_thread_ind)
	, world(rhs.world)
	, open(rhs.open)
{
	assert(type == END_EFFECTOR);
	
	assert(((thread == NULL) && (constrained_vertex_num == -1)) || ((thread != NULL) && (constrained_vertex_num != -1)));
	if (thread == NULL) {
		constraint_ind = -1;
	} else {
		vector<int> constrained_vertices_nums;
		thread->getConstrainedVerticesNums(constrained_vertices_nums);
		constraint_ind = find(constrained_vertices_nums, constraint);
		assert(constraint_ind != -1);
	}
	
	Intersection_Object* short_handle = new Intersection_Object();
	short_handle->_radius = short_handle_r;
  i_objs.push_back(short_handle);
  
	Intersection_Object* handle = new Intersection_Object();
	handle->_radius = handle_r;
  i_objs.push_back(handle);
  
  for (int piece=0; piece<pieces; piece++) {
 		Intersection_Object* tip_piece = new Intersection_Object();
 		tip_piece->_radius = 0.9+((double) piece)*((handle_r)-0.9)/((double) pieces-1);
  	i_objs.push_back(tip_piece);
	}
  
  for (int piece=0; piece<pieces; piece++) {
 		Intersection_Object* tip_piece = new Intersection_Object();
 		tip_piece->_radius = 0.9+((double) piece)*((handle_r)-0.9)/((double) pieces-1);
  	i_objs.push_back(tip_piece);
	}
	
	setTransform(rhs.position, rhs.rotation);
}*/

//backup info is not passed. user of this object should call again backup() if needed.
EndEffector::EndEffector(const EndEffector& rhs, World* w)
	: EnvObject(rhs.color0, rhs.color1, rhs.color2, rhs.type)
	, thread(rhs.thread)
	, constraint(rhs.constraint)
	, needle(NULL)
	, world(w)
	, open(rhs.open)
{
	assert(type == END_EFFECTOR);
	
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

	btCapsuleShapeX* capsule_short_handle = new btCapsuleShapeX(btScalar(short_handle_r+REPULSION_DIST), btScalar(short_handle_h));
	capsule_short_handle->setMargin(0.f);
	btCollisionObject* short_handle = new btCollisionObject();
  short_handle->setCollisionShape(capsule_short_handle);
  short_handle->setUserPointer(this);
	col_objs.push_back(short_handle);
  
	btCapsuleShapeX* capsule_handle = new btCapsuleShapeX(btScalar(handle_r+4.0*REPULSION_DIST), btScalar(handle_h));
	capsule_handle->setMargin(0.f);
	btCollisionObject* handle = new btCollisionObject();
  handle->setCollisionShape(capsule_handle);
  handle->setUserPointer(this);
  if (world->collision_world != NULL)
		world->collision_world->addCollisionObject(handle);
  col_objs.push_back(handle);

  for (int piece=0; piece<pieces; piece++) {
		btCapsuleShapeX* capsule_tip_piece = new btCapsuleShapeX(btScalar(0.9+(double(piece))*(handle_r-0.9)/(pieces-1.0)+REPULSION_DIST), btScalar(h));
		capsule_tip_piece->setMargin(0.f);
		btCollisionObject* tip_piece = new btCollisionObject();
  	tip_piece->setCollisionShape(capsule_tip_piece);
  	tip_piece->setUserPointer(this);
  	if (world->collision_world != NULL)
			world->collision_world->addCollisionObject(tip_piece);
  	col_objs.push_back(tip_piece);
	}
  
  for (int piece=0; piece<pieces; piece++) {
		btCapsuleShapeX* capsule_tip_piece = new btCapsuleShapeX(btScalar(0.9+(double(piece))*(handle_r-0.9)/(pieces-1.0)+REPULSION_DIST), btScalar(h));
		capsule_tip_piece->setMargin(0.f);
		btCollisionObject* tip_piece = new btCollisionObject();
  	tip_piece->setCollisionShape(capsule_tip_piece);
  	tip_piece->setUserPointer(this);
  	if (world->collision_world != NULL)
			world->collision_world->addCollisionObject(tip_piece);
  	col_objs.push_back(tip_piece);
	}

	setTransform(rhs.position, rhs.rotation);
}

EndEffector::~EndEffector()
{
	for (int i=0; i<col_objs.size(); i++) {
		if (world->collision_world != NULL)
			world->collision_world->removeCollisionObject(col_objs[i]);
		delete col_objs[i];
		col_objs[i] = NULL;
	}
}

void EndEffector::writeToFile(ofstream& file)
{
	assert(type == END_EFFECTOR);
	file << type << " ";
	for (int i=0; i<3; i++)
		file << position(i) << " ";
	for (int r=0; r < 3; r++)
    for (int c=0; c < 3; c++)
      file << rotation(r,c) << " ";
  file << constraint << " " << world->objectIndex<ThreadConstrained>(thread) << " " << open << " ";
  file << "\n";
}

EndEffector::EndEffector(ifstream& file, World* w)
	: EnvObject(default_color0, default_color1, default_color2, END_EFFECTOR)
	, thread(NULL)
	, needle(NULL)
	, world(w)
{
	for (int i=0; i<3; i++)
		file >> position(i);
	for (int r=0; r < 3; r++)
    for (int c=0; c < 3; c++)
      file >> rotation(r,c);
  
  int world_thread_ind;
  file >> constraint >> world_thread_ind >> open;
 	thread = world->objectAtIndex<ThreadConstrained>(world_thread_ind);
 	updateConstraintIndex();
	assert(((thread == NULL) && (constraint == -1) && (constraint_ind == -1)) || 
				 ((thread != NULL) && (constraint != -1) && (constraint_ind != -1)));

	btCapsuleShapeX* capsule_short_handle = new btCapsuleShapeX(btScalar(short_handle_r+REPULSION_DIST), btScalar(short_handle_h));
	capsule_short_handle->setMargin(0.f);
	btCollisionObject* short_handle = new btCollisionObject();
  short_handle->setCollisionShape(capsule_short_handle);
  short_handle->setUserPointer(this);
  col_objs.push_back(short_handle);
  
	btCapsuleShapeX* capsule_handle = new btCapsuleShapeX(btScalar(handle_r+4.0*REPULSION_DIST), btScalar(handle_h));
	capsule_handle->setMargin(0.f);
	btCollisionObject* handle = new btCollisionObject();
  handle->setCollisionShape(capsule_handle);
  handle->setUserPointer(this);
  if (world->collision_world != NULL)
		world->collision_world->addCollisionObject(handle);
  col_objs.push_back(handle);

  for (int piece=0; piece<pieces; piece++) {
		btCapsuleShapeX* capsule_tip_piece = new btCapsuleShapeX(btScalar(0.9+(double(piece))*(handle_r-0.9)/(pieces-1.0)+REPULSION_DIST), btScalar(h));
		capsule_tip_piece->setMargin(0.f);
		btCollisionObject* tip_piece = new btCollisionObject();
  	tip_piece->setCollisionShape(capsule_tip_piece);
  	tip_piece->setUserPointer(this);
  	if (world->collision_world != NULL)
			world->collision_world->addCollisionObject(tip_piece);
  	col_objs.push_back(tip_piece);
	}
  
  for (int piece=0; piece<pieces; piece++) {
		btCapsuleShapeX* capsule_tip_piece = new btCapsuleShapeX(btScalar(0.9+(double(piece))*(handle_r-0.9)/(pieces-1.0)+REPULSION_DIST), btScalar(h));
		capsule_tip_piece->setMargin(0.f);
		btCollisionObject* tip_piece = new btCollisionObject();
  	tip_piece->setCollisionShape(capsule_tip_piece);
  	tip_piece->setUserPointer(this);
  	if (world->collision_world != NULL)
			world->collision_world->addCollisionObject(tip_piece);
  	col_objs.push_back(tip_piece);
	}
	
	setTransform(position, rotation);
}

void EndEffector::setTransform(const Vector3d& pos, const Matrix3d& rot, bool limit_displacement, double max_displacement, double max_angle_change)
{
	if (limit_displacement) {
		Vector3d displacement = pos - position;
		//double displacement_norm = displacement.norm();
		double max_coor_displacement = max(max(abs(displacement(0)), abs(displacement(1))), abs(displacement(2)));
		
		double angle_change	= angle_between(rot.col(0), rotation.col(0));
				
		Vector3d old_pos = position;
		
		Quaterniond new_q(rot);
		Quaterniond last_q(rotation);
		if (max_coor_displacement > max_displacement) {
			position = position + (max_displacement/max_coor_displacement) * (pos-position);
		} else {
			position = pos;
		}
		if (angle_change > max_angle_change) {
			Quaterniond interp_q = last_q.slerp(max_angle_change/angle_change, new_q);
			rotation = interp_q.toRotationMatrix(); 
		} else {
			rotation = rot;
		}
		rotation = (Matrix3d) AngleAxisd(rotation); // orthonormalizes the rotation matrix due to numerical errors
		
		Vector3d disp = old_pos - position;
		if ((abs(disp(0)) > 0.2000001) || (abs(disp(1)) > 0.2000001) || (abs(disp(2)) > 0.2000001)) {
			cout << disp.transpose() << endl;
		}
		
	} else {
		position = pos;
    rotation = rot;
  }
  
  if(isThreadAttached()) {
		thread->updateConstrainedTransform(constraint_ind, position, rotation);
		//thread->minimize_energy();
	}
	if(isNeedleAttached()) {
		needle->setTransformFromEndEffector(position, rotation);		
	}
	
	updateCollisionObjects();
}

void EndEffector::updateCollisionObjects()
{
	Vector3d new_pos;
	Matrix3d new_rot;
	Matrix3d open_rot = (Matrix3d) AngleAxisd(-(open?15.0:0.0)*M_PI/180, rotation*Vector3d::UnitZ());
	
//	col_objs[0]->getWorldTransform().setOrigin(tobtVector3(rotation * Vector3d(grab_offset-1.5, 0.0, 0.0) + position));
//	col_objs[0]->getWorldTransform().setBasis(tobtMatrix3x3(rotation));
	
	col_objs[1]->getWorldTransform().setOrigin(tobtVector3(rotation * Vector3d(end+15.0, 0.0, 0.0) + position));
	col_objs[1]->getWorldTransform().setBasis(tobtMatrix3x3(rotation));
	
	for (int piece=0; piece<pieces; piece++) {
		double r = 0.9+(double(piece))*(handle_r-0.9)/(pieces-1.0);
		new_rot = open_rot * rotation;
		new_pos = open_rot * rotation * Vector3d(-end, 0.0, 0.0) + rotation * Vector3d(end, 0.0, 0.0) + position;
		col_objs[piece+2]->getWorldTransform().setOrigin(tobtVector3(new_rot * Vector3d(start+(double(piece)+0.5)*h, r, 0.0) + new_pos));
		col_objs[piece+2]->getWorldTransform().setBasis(tobtMatrix3x3(new_rot));
		
		new_rot = open_rot.transpose() * rotation;
		new_pos = open_rot.transpose() * rotation * Vector3d(-end, 0.0, 0.0) + rotation * Vector3d(end, 0.0, 0.0) + position;
		col_objs[piece+(int)pieces+2]->getWorldTransform().setOrigin(tobtVector3(new_rot * Vector3d(start+(double(piece)+0.5)*h, -r, 0.0) + new_pos));
		col_objs[piece+(int)pieces+2]->getWorldTransform().setBasis(tobtMatrix3x3(new_rot));
	}
}

void EndEffector::updateTransformFromAttachment()
{
	if (isThreadAttached()) {
//		position = thread->positionAtConstraint(constraint_ind);
//		rotation = thread->rotationAtConstraint(constraint_ind);
		setTransform(thread->positionAtConstraint(constraint_ind), thread->rotationAtConstraint(constraint_ind), true);
		updateCollisionObjects();
	}
	if (isNeedleAttached()) {
		vector<Box*> boxes;
		world->getObjects<Box>(boxes);
		bool has_box_needle = false;
		for (int i = 0; i < boxes.size(); i++) {
			if (boxes[i]->isNeedleAttached() && (boxes[i]->getNeedle() == needle)) {
				has_box_needle = true;
				break;
			}
		}
		if (!has_box_needle) {
			needle->updateTransformFromAttachment();
		}
		needle->getEndEffectorTransform(position, rotation);
		updateCollisionObjects();
	}
}

void EndEffector::draw()
{
	glColor3f(color0, color1, color2);
#ifndef PICTURE
	for (int i = 1; i < 8; i++) {
		if (i == 2 || i == 5) { continue; }
		drawCapsule(col_objs[i]);
	}
	glColor3f(0.0, 0.5, 0.0);
	drawCapsule(col_objs[2]);
	drawCapsule(col_objs[5]);
#else
	for (int i = 1; i < 8; i++) {
		drawCapsule(col_objs[i]);
	}
	glColor3f(0.0, 0.5, 0.0);
	drawCapsule(col_objs[2]);
	drawCapsule(col_objs[5]);
#endif
}

void EndEffector::drawDebug()
{
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	glColor3f(0.8, 0.8, 0.8);

	Vector3d origin = toVector3d(col_objs[1]->getWorldTransform().getOrigin());
	float half_height = handle_h/2.0;
	Vector3d axis = toMatrix3d(col_objs[1]->getWorldTransform().getBasis()).col(0);
	Vector3d start_pos = origin + half_height * axis;
	Vector3d end_pos = origin - (half_height+9.0) * axis;

	glVertex3d(start_pos(0), start_pos(1), start_pos(2));
	glVertex3d(end_pos(0), end_pos(1), end_pos(2));
	glEnd();
	glEnable(GL_LIGHTING);
}

void EndEffector::updateConstraint()
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

void EndEffector::updateConstraintIndex()
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

void EndEffector::backup()
{
	backup_position = position;
	backup_rotation = rotation;
	backup_constraint = constraint;
	backup_thread_ind = world->objectIndex<ThreadConstrained>(thread);
	backup_open = open;
}

// caller is responsible for having backedup before restoring
void EndEffector::restore()
{
	setTransform(backup_position, backup_rotation);
	constraint = backup_constraint;
	thread = world->objectAtIndex<ThreadConstrained>(backup_thread_ind);
	updateConstraintIndex();
	open = backup_open;
}

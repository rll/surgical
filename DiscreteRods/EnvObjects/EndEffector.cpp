#include "EndEffector.h"
#include "World.h"
#include "../threadpiece_discrete.h" //TODO for repulsion coefficient. should be removed if collision is checked in world.
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
	i_objs.clear();
	
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
}

EndEffector::~EndEffector()
{
	for (int i=0; i<i_objs.size(); i++) {
		delete i_objs[i];
		i_objs[i] = NULL;
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
	
	setTransform(position, rotation);
}

void EndEffector::setTransform(const Vector3d& pos, const Matrix3d& rot, bool limit_displacement, double max_displacement, double max_angle_change)
{
	if (limit_displacement) {
		double displacement = (pos-position).norm();
		//double angle_change	= 2*asin((rot.col(0) - rotation.col(0)).norm()/2);
		double angle_change	= angle_between(rot.col(0), rotation.col(0));
		
		Quaterniond new_q(rot);
		Quaterniond last_q(rotation);
		if (displacement > max_displacement) {
			position = position + (max_displacement/displacement) * (pos-position);
		} else {
			position = pos;
		}
		if (angle_change > max_angle_change) {
			Quaterniond interp_q = last_q.slerp(max_angle_change/angle_change, new_q);
			rotation = interp_q.toRotationMatrix(); 
		} else {
			rotation = rot;
		}
		rotation.col(1) = rotation.col(2).cross(rotation.col(0));
		rotation.col(2) = rotation.col(0).cross(rotation.col(1));
		rotation.col(0).normalize();
		rotation.col(1).normalize();
		rotation.col(2).normalize();
	} else {
		position = pos;
    rotation = rot;
  }
  
  if(isThreadAttached()) {
		thread->updateConstrainedTransform(constraint_ind, position, rotation);
		//thread->minimize_energy();
	}
	if(isNeedleAttached()) {
		vector<Box*> boxes;
		world->getObjects<Box>(boxes);
		Box* box = NULL;
		for (int i = 0; i < boxes.size(); i++) {
			if (boxes[i]->isNeedleAttached() && (boxes[i]->getNeedle() == needle)) {
				box = boxes[i];
				break;
			}
		}
		
		if (box!=NULL) {	//box has needle
			if (box->constraint0!=-1 && box->constraint1!=-1) {
			
			} else if (box->constraint0!=-1) {
				
			}
			
			needle->setTransformFromEndEffectorBoxConstrained(position, rotation);
			//cout << "party_in_the_box" << endl;
			
		} else {
			needle->setTransformFromEndEffector(position, rotation);		
			
		}

		
	}
	
	updateIntersectionObjects();
}

void EndEffector::updateIntersectionObjects()
{
	Vector3d start_pos;
	Vector3d end_pos;
	Vector3d new_pos;
	Matrix3d new_rot;
	Matrix3d open_rot = (Matrix3d) AngleAxisd(-(open?15.0:0.0)*M_PI/180, rotation*Vector3d::UnitZ());
	
	start_pos = rotation * Vector3d(grab_offset-3.0, 0.0, 0.0) + position;
	end_pos = rotation * Vector3d(grab_offset, 0.0, 0.0) + position;
	i_objs[0]->_start_pos = start_pos;
	i_objs[0]->_end_pos 	= end_pos;
	
	start_pos = rotation * Vector3d(end, 0.0, 0.0) + position;
	end_pos = rotation * Vector3d(end+30.0, 0.0, 0.0) + position;
	i_objs[1]->_start_pos = start_pos;
	i_objs[1]->_end_pos 	= end_pos;
	
	for (int piece=0; piece<pieces; piece++) {
		double r = i_objs[piece+pieces+2]->_radius;
		new_rot = open_rot * rotation;
		new_pos = open_rot * rotation * Vector3d(-end, 0.0, 0.0) + rotation * Vector3d(end, 0.0, 0.0) + position;
		start_pos = new_rot * Vector3d(start+((double) piece)*h, r, 0.0) + new_pos;
		end_pos = new_rot * Vector3d(start+((double) piece+1)*h, r, 0.0) + new_pos;
 		i_objs[piece+2]->_start_pos = start_pos;
		i_objs[piece+2]->_end_pos 	= end_pos;
	}
	
	for (int piece=0; piece<pieces; piece++) {
		double r = i_objs[piece+pieces+2]->_radius;
		new_rot = open_rot.transpose() * rotation;
		new_pos = open_rot.transpose() * rotation * Vector3d(-end, 0.0, 0.0) + rotation * Vector3d(end, 0.0, 0.0) + position;
		start_pos = new_rot * Vector3d(start+((double) piece)*h, -r, 0.0) + new_pos;
		end_pos = new_rot * Vector3d(start+((double) piece+1)*h, -r, 0.0) + new_pos;
  	i_objs[piece+pieces+2]->_start_pos = start_pos;
		i_objs[piece+pieces+2]->_end_pos 	= end_pos;;
	}
}

void EndEffector::updateTransformFromAttachment()
{
	if (isThreadAttached()) {
		position = thread->positionAtConstraint(constraint_ind);
		rotation = thread->rotationAtConstraint(constraint_ind);
		updateIntersectionObjects();
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
		updateIntersectionObjects();
	}
}

void EndEffector::draw()
{
	glColor3f(color0, color1, color2);
	drawCylinder(i_objs[1]->_start_pos, i_objs[1]->_end_pos, i_objs[1]->_radius);
	drawSphere(i_objs[1]->_start_pos, i_objs[1]->_radius);
	drawSphere(i_objs[1]->_end_pos, i_objs[1]->_radius);
 	
 	int obj_ind;
  for (obj_ind = 3; obj_ind<2+pieces-1; obj_ind++) {
 		drawCylinder(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_end_pos, i_objs[obj_ind]->_radius);
		drawSphere(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_radius);
	}	
	drawCylinder(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_end_pos, i_objs[obj_ind]->_radius);
	drawSphere(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_radius);
	drawSphere(i_objs[obj_ind]->_end_pos, i_objs[obj_ind]->_radius);
	
	for (obj_ind+=2; obj_ind<2+2*pieces-1; obj_ind++) {
 		drawCylinder(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_end_pos, i_objs[obj_ind]->_radius);
		drawSphere(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_radius);
	}	
	drawCylinder(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_end_pos, i_objs[obj_ind]->_radius);
	drawSphere(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_radius);
	drawSphere(i_objs[obj_ind]->_end_pos, i_objs[obj_ind]->_radius);
  
  glColor3f(0.3, 0.3, 0.0);
  drawCylinder(i_objs[2]->_start_pos, i_objs[2]->_end_pos, i_objs[2]->_radius);
	drawSphere(i_objs[2]->_start_pos, i_objs[2]->_radius);
	drawCylinder(i_objs[5]->_start_pos, i_objs[5]->_end_pos, i_objs[5]->_radius);
	drawSphere(i_objs[5]->_start_pos, i_objs[5]->_radius);
  
  //capsule representing possible attachment of cursor
//  glColor3f(0.3, 0.3, 0.0);
//  drawCylinder(i_objs[0]->_start_pos, i_objs[0]->_end_pos, i_objs[0]->_radius);
//	drawSphere(i_objs[0]->_start_pos, i_objs[0]->_radius);
//	drawSphere(i_objs[0]->_end_pos, i_objs[0]->_radius);
	
	//To visualize the collision capsule
	//const Vector3d start = i_objs[1]->_start_pos + 2.0*(i_objs[1]->_start_pos - i_objs[1]->_end_pos).normalized();
	//const Vector3d end = i_objs[1]->_end_pos;
	//drawCylinder(start, end, 1.2*i_objs[1]->_radius);
	//drawSphere(start, 1.2*i_objs[1]->_radius);
	//drawSphere(end, 1.2*i_objs[1]->_radius);
}

void EndEffector::drawDebug()
{
	return;
	
	//To visualize the collision capsule
	glColor3f(0,1,0);
	const Vector3d start = i_objs[1]->_start_pos + 2.0*(i_objs[1]->_start_pos - i_objs[1]->_end_pos).normalized();
	const Vector3d end = i_objs[1]->_end_pos;
	drawCylinder(start, end, 1.2*i_objs[1]->_radius);
	drawSphere(start, 1.2*i_objs[1]->_radius);
	drawSphere(end, 1.2*i_objs[1]->_radius);
	
	Vector3d direction;
	vector<Box*> boxes;
	world->getObjects<Box>(boxes);
	vector<EndEffector*> ee_effs;
	world->getObjects<EndEffector>(ee_effs);
	if (boxes.size() > 0 && this == ee_effs[2]) {
		double dist = capsuleBoxDistance(start, end, 1.2*i_objs[1]->_radius, boxes[0]->getPosition(), boxes[0]->getHalfLength(), direction);
		//if (dist < 0) {
			cout << dist << endl;
			glColor3f(0,0,1);
			//drawSphere(positionWorldOnA, 2.0);		
			//drawSphere(positionWorldOnB, 2.0);
			drawArrow(Vector3d::Zero(), direction);
		//}
	}
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

bool EndEffector::capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections)
{
	/*
	bool found = false;
  Vector3d direction;
  for (int i=0; i < i_objs.size(); i++) {
    double intersection_dist = capsuleCapsuleDistance(start, end, radius, i_objs[i]->_start_pos, i_objs[i]->_end_pos, i_objs[i]->_radius, direction);
    if(intersection_dist < 0) {
      found = true;
      if (i==1) {		// if the handle is intersecting and the part intersecting is the one close to the grippers, then this case have to be treated differently because the thread can oscillate between this part of the handle and the grippers in an infinite loop.
      	Vector3d handle_direction;
      	double handle_intersection_dist = capsuleSphereDistance(start, end, radius, i_objs[i]->_start_pos, i_objs[i]->_radius, handle_direction);
      	if (handle_intersection_dist < 0) {
      		handle_direction = (i_objs[i]->_start_pos-i_objs[i]->_end_pos).normalized();
      		intersections.push_back(Intersection(capsule_ind, -handle_intersection_dist, handle_direction));
      	} else {
      		intersections.push_back(Intersection(capsule_ind, -intersection_dist, direction));
      	}
      } else {
      	intersections.push_back(Intersection(capsule_ind, -intersection_dist, direction));
      }
      //cout << i << " ";
    }
  }
  //if (found) cout << endl;
  return found;
 */
  Vector3d direction;
  double intersection_dist = capsuleCapsuleDistance(start, end, radius, i_objs[1]->_start_pos + 9.0*(i_objs[1]->_start_pos - i_objs[1]->_end_pos).normalized(), i_objs[1]->_end_pos, i_objs[1]->_radius, direction);
  if(intersection_dist < 0) {
    intersections.push_back(Intersection(capsule_ind, -intersection_dist, direction));
    return true;
  }  
  return false;
}

double EndEffector::capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius)
{
	/*
	if (REPULSION_COEFF <= 0.0) { return 0.0; }
	double energy = 0.0;
	Vector3d direction;
	for (int i=0; i < i_objs.size(); i++) {
		double dist = capsuleCapsuleDistance(start, end, radius, i_objs[i]->_start_pos, i_objs[i]->_end_pos, i_objs[i]->_radius, direction);
		if (dist < 0 || dist > radius)
			continue;
		energy += REPULSION_COEFF/2.0 * pow(dist-radius,2);
	}
	return energy;
	*/
//	if (REPULSION_COEFF <= 0.0) { return 0.0; }
//	Vector3d direction;
//	double dist = capsuleCapsuleDistance(start, end, radius, i_objs[1]->_start_pos + 9.0*(i_objs[1]->_start_pos - i_objs[1]->_end_pos).normalized(), i_objs[1]->_end_pos, i_objs[1]->_radius, direction);
//	if (dist > radius)
//		return 0.0;
//	return REPULSION_COEFF/2.0 * pow(dist-radius,2);
//	
//	
	if (REPULSION_COEFF <= 0.0) { return 0.0; }
	Vector3d direction;
	const double dist = capsuleCapsuleDistance(start, end, radius, i_objs[1]->_start_pos + 9.0*(i_objs[1]->_start_pos - i_objs[1]->_end_pos).normalized(), i_objs[1]->_end_pos, i_objs[1]->_radius, direction);
	return repulsionEnergy(dist, radius, direction);
	
	//dist < 0 || 
	
//	if (dist < 0) {
//		return REPULSION_COEFF/2.0 * pow(dist-radius,2);
//	} else if (dist <= radius) {
//		return REPULSION_COEFF/2.0 * pow(dist-radius,2);
//	}
//	return 0.0;
}

void EndEffector::capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient)
{
	/*
	if (REPULSION_COEFF <= 0.0) { return; }
	Vector3d direction;
	for (int i=0; i < i_objs.size(); i++) {
		double dist = capsuleCapsuleDistance(start, end, radius, i_objs[i]->_start_pos, i_objs[i]->_end_pos, i_objs[i]->_radius, direction);
		if (dist < 0 || dist > radius)
			continue;
		gradient -= REPULSION_COEFF * (radius - dist) * direction.normalized();
	}
	*/
	
	if (REPULSION_COEFF <= 0.0) { return; }
	Vector3d direction;
	const double dist = capsuleCapsuleDistance(start, end, radius, i_objs[1]->_start_pos + 9.0*(i_objs[1]->_start_pos - i_objs[1]->_end_pos).normalized(), i_objs[1]->_end_pos, i_objs[1]->_radius, direction);
	gradient -= repulsionEnergyGradient(dist, radius, direction);
	
//	if (REPULSION_COEFF <= 0.0) { return; }
//	Vector3d direction;
//	double dist = capsuleCapsuleDistance(start, end, radius, i_objs[1]->_start_pos + 9.0*(i_objs[1]->_start_pos - i_objs[1]->_end_pos).normalized(), i_objs[1]->_end_pos, i_objs[1]->_radius, direction);
//	if (dist < 0) {					
//		gradient -= REPULSION_COEFF * (radius - dist) * direction.normalized();
//	} else if (dist <= radius) {
//		gradient -= REPULSION_COEFF * (radius - dist) * direction.normalized();
//	}
}

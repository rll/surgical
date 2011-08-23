#include "EndEffector.h"
#include "../threadpiece_discrete.h"
#include "../ThreadConstrained.h"

EndEffector::EndEffector(const Vector3d& pos, const Matrix3d& rot)
	: EnvObject(pos, rot, 0.7, 0.7, 0.7, END_EFFECTOR)
	, degrees(0.0)
	, constraint(-1)
	, constraint_ind(-1)
	, thread(NULL)
	, attachment(NULL)
{
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
	
	recomputeFromTransform(pos, rot);
}

EndEffector::~EndEffector()
{
	for (int i=0; i<i_objs.size(); i++) {
		delete i_objs[i];
		i_objs[i] = NULL;
	}
}

//updateIndFromPointers(World* world) should have been called before calling this
void EndEffector::writeToFile(ofstream& file)
{
	file << type << " ";
	for (int i=0; i<3; i++)
		file << position(i) << " ";
	for (int r=0; r < 3; r++)
  {
    for (int c=0; c < 3; c++)
    {
      file << rotation(r,c) << " ";
    }
  }
  file << degrees << " " << constraint << " " << constraint_ind << " " << thread_ind << " " << attachment_ind << " ";
  file << "\n";
}

//linkPointersFromInd(World* world) should be called after calling this
EndEffector::EndEffector(ifstream& file)
	: thread(NULL)
	, attachment(NULL)
{
	color0 = color1 = color2 = 0.7;
  type = END_EFFECTOR;
	
	for (int i=0; i<3; i++)
		file >> position(i);
	for (int r=0; r < 3; r++)
  {
    for (int c=0; c < 3; c++)
    {
      file >> rotation(r,c);
    }
  }
  
  file >> degrees >> constraint >> constraint_ind >> thread_ind >> attachment_ind;

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
	
	recomputeFromTransform(position, rotation);
}

void EndEffector::updateIndFromPointers(World* world)
{
	if (thread == NULL) {
		thread_ind = -1;
	} else {
		vector<ThreadConstrained*> threads = *(world->getThreads());
		for (thread_ind = 0; thread_ind<threads.size(); thread_ind++) {
			if (threads[thread_ind] == thread)
				break;
		}
		if (thread_ind == threads.size())
			cout << "Internal error: EndEffector::updateIndFromPointers: thread was not in *(world->getThreads())." << endl;
	}
	
	if (attachment == NULL) {
		attachment_ind = -1;
	} else {
		vector<EnvObject*> cursors = world->getEnvObjs(CURSOR);
		for (attachment_ind = 0; attachment_ind<cursors.size(); attachment_ind++) {
			Cursor* cursor = dynamic_cast<Cursor*>(cursors[attachment_ind]);
			if (cursor == attachment)
				break;
		}
		if (attachment_ind == cursors.size())
			cout << "Internal error: EndEffector::updateIndFromPointers: attachment was not in world->getEnvObjs(CURSOR)." << endl;
	}
}

void EndEffector::linkPointersFromInd(World* world)
{
	if (thread_ind == -1) {
		thread = NULL;
	} else {
		vector<ThreadConstrained*> threads = *(world->getThreads());
		if (thread_ind < 0 || thread_ind >= threads.size())
			cout << "Internal error: EndEffector::linkPointersFromInd: thread_ind is out of bounds." << endl;
		thread = threads[thread_ind];
	}
	
	if (attachment_ind == -1) {
		attachment = NULL;
	} else {
		vector<EnvObject*> cursors = world->getEnvObjs(CURSOR);
		if (attachment_ind < 0 || attachment_ind >= cursors.size())
			cout << "Internal error: EndEffector::linkPointersFromInd: attachment_ind is out of bounds." << endl;
		attachment = dynamic_cast<Cursor*>(cursors[attachment_ind]);
	}
}

void EndEffector::recomputeFromTransform(const Vector3d& pos, const Matrix3d& rot)
{
	if (attachment != NULL) {
		degrees = attachment->isOpen() ? 15.0 : 0.0;
	} else
		degrees = 0.0;
	Vector3d start_pos;
	Vector3d end_pos;
	Vector3d new_pos;
	Matrix3d new_rot;
	Matrix3d open_rot = (Matrix3d) AngleAxisd(-degrees*M_PI/180, rot*Vector3d::UnitZ());
	
	start_pos = rot * Vector3d(grab_offset-3.0, 0.0, 0.0) + pos;
	end_pos = rot * Vector3d(grab_offset, 0.0, 0.0) + pos;
	i_objs[0]->_start_pos = start_pos;
	i_objs[0]->_end_pos 	= end_pos;
	
	start_pos = rot * Vector3d(end, 0.0, 0.0) + pos;
	end_pos = rot * Vector3d(end+30.0, 0.0, 0.0) + pos;
	i_objs[1]->_start_pos = start_pos;
	i_objs[1]->_end_pos 	= end_pos;
	
	for (int piece=0; piece<pieces; piece++) {
		double r = i_objs[piece+pieces+2]->_radius;
		new_rot = open_rot * rot;
		new_pos = open_rot * rot * Vector3d(-end, 0.0, 0.0) + rot * Vector3d(end, 0.0, 0.0) + pos;
		start_pos = new_rot * Vector3d(start+((double) piece)*h, r, 0.0) + new_pos;
		end_pos = new_rot * Vector3d(start+((double) piece+1)*h, r, 0.0) + new_pos;
 		i_objs[piece+2]->_start_pos = start_pos;
		i_objs[piece+2]->_end_pos 	= end_pos;
	}
	
	for (int piece=0; piece<pieces; piece++) {
		double r = i_objs[piece+pieces+2]->_radius;
		new_rot = open_rot.transpose() * rot;
		new_pos = open_rot.transpose() * rot * Vector3d(-end, 0.0, 0.0) + rot * Vector3d(end, 0.0, 0.0) + pos;
		start_pos = new_rot * Vector3d(start+((double) piece)*h, -r, 0.0) + new_pos;
		end_pos = new_rot * Vector3d(start+((double) piece+1)*h, -r, 0.0) + new_pos;
  	i_objs[piece+pieces+2]->_start_pos = start_pos;
		i_objs[piece+pieces+2]->_end_pos 	= end_pos;;
	}
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
	if (REPULSION_COEFF <= 0.0) { return 0.0; }
	Vector3d direction;
	double dist = capsuleCapsuleDistance(start, end, radius, i_objs[1]->_start_pos + 9.0*(i_objs[1]->_start_pos - i_objs[1]->_end_pos).normalized(), i_objs[1]->_end_pos, i_objs[1]->_radius, direction);
	if (dist < 0 || dist > radius)
		return 0.0;
	return REPULSION_COEFF/2.0 * pow(dist-radius,2);
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
	double dist = capsuleCapsuleDistance(start, end, radius, i_objs[1]->_start_pos + 9.0*(i_objs[1]->_start_pos - i_objs[1]->_end_pos).normalized(), i_objs[1]->_end_pos, i_objs[1]->_radius, direction);
	if (dist < 0 || dist > radius)
		return;
	gradient -= REPULSION_COEFF * (radius - dist) * direction.normalized();
}

void EndEffector::draw()
{
  for (int obj_ind = 1; obj_ind < i_objs.size(); obj_ind++) {
		drawCylinder(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_end_pos, i_objs[obj_ind]->_radius, color0, color1, color2);
		drawSphere(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_radius, color0, color1, color2);
		drawSphere(i_objs[obj_ind]->_end_pos, i_objs[obj_ind]->_radius, color0, color1, color2);
  }
  
  drawCylinder(i_objs[0]->_start_pos, i_objs[0]->_end_pos, i_objs[0]->_radius, 0.3, 0.3, 0.0);
	drawSphere(i_objs[0]->_start_pos, i_objs[0]->_radius, 0.3, 0.3, 0.0);
	drawSphere(i_objs[0]->_end_pos, i_objs[0]->_radius, 0.3, 0.3, 0.0);
	
	/*
	drawCylinder(i_objs[1]->_start_pos + 9.0*(i_objs[1]->_start_pos - i_objs[1]->_end_pos).normalized(), i_objs[1]->_end_pos, i_objs[1]->_radius, 0.0, 1.0, 0.0);
	drawSphere(i_objs[1]->_start_pos + 9.0*(i_objs[1]->_start_pos - i_objs[1]->_end_pos).normalized(), i_objs[1]->_radius, 0.0, 1.0, 0.0);
	drawSphere(i_objs[1]->_end_pos, i_objs[1]->_radius, 0.0, 1.0, 0.0);
	*/
}

void EndEffector::attach(Cursor* cursor)
{
	degrees = cursor->isOpen() ? 15.0 : 0.0;
	attachment = cursor;
}

void EndEffector::dettach()
{
	if (attachment == NULL)
		cout << "Internal errror: EndEffector::dettach(): end effector cannot dettach since it does't have a cursor attached" << endl;
	attachment = NULL;
}

void EndEffector::updateConstraint()
{
	if (constraint_ind != -1) {
		if (thread == NULL)
			cout << "Internal errror: EndEffector::updateConstraint(): thread should not be NULL because constraint_ind!=-1" << endl;
		vector<int> constrained_vertices_nums;
		thread->getConstrainedVerticesNums(constrained_vertices_nums);
		constraint = constrained_vertices_nums[constraint_ind];
	}
}

void EndEffector::updateConstraintIndex()
{
	if (constraint != -1) {
		if (thread == NULL)
			cout << "Internal errror: EndEffector::updateConstraintIndex(): thread should not be NULL because constraint!=-1" << endl;
		vector<int> constrained_vertices_nums;
		thread->getConstrainedVerticesNums(constrained_vertices_nums);
		constraint_ind = find(constrained_vertices_nums, constraint);
		if (constraint_ind == -1)
			cout << "Internal errror: EndEffector::updateConstraintIndex(): constraint is supposed to be in constrained_vertices_nums but it isn't" << endl;
	}
}

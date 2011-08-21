#include "EndEffector.h"
#include "../threadpiece_discrete.h"

EndEffector::EndEffector(const Vector3d& pos, const Matrix3d& rot)
	: EnvObject(pos, rot, 0.7, 0.7, 0.7)
	, degrees(0.0)
	, constraint(-1)
	, constraint_ind(-1)
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
 		tip_piece->_radius = 0.5+((double) piece)*((0.8*handle_r)-0.5)/((double) pieces-1);
  	i_objs.push_back(tip_piece);
	}
  
  for (int piece=0; piece<pieces; piece++) {
 		Intersection_Object* tip_piece = new Intersection_Object();
 		tip_piece->_radius = 0.5+((double) piece)*((0.8*handle_r)-0.5)/((double) pieces-1);
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
	bool found = false;
  Vector3d direction;
  for (int i=0; i < i_objs.size(); i++) {
    double intersection_dist = capsuleCapsuleDistance(start, end, radius, i_objs[i]->_start_pos, i_objs[i]->_end_pos, i_objs[i]->_radius, direction);
    if(intersection_dist < 0) {
      found = true;
      intersections.push_back(Intersection(capsule_ind, -intersection_dist, direction));
    }
  }
  return found;
}

double EndEffector::capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius)
{
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
}

void EndEffector::capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient)
{
	if (REPULSION_COEFF <= 0.0) { return; }
	Vector3d direction;
	for (int i=0; i < i_objs.size(); i++) {
		double dist = capsuleCapsuleDistance(start, end, radius, i_objs[i]->_start_pos, i_objs[i]->_end_pos, i_objs[i]->_radius, direction);
		if (dist < 0 || dist > radius)
			continue;
		gradient -= REPULSION_COEFF * (radius - dist) * direction.normalized();
	}
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

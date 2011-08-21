#include "Cursor.h"
#include "../threadpiece_discrete.h"

Cursor::Cursor(const Vector3d& pos, const Matrix3d& rot)
	: EnvObject(pos, rot, 0.0, 0.0, 0.0)
	, end_eff(NULL)
	, height(3)
	, radius(2)
	,	attach_dettach_attempt(false)
	, open(false)
	, last_open(false)
{
	i_obj = new Intersection_Object();
	i_obj->_radius = radius;
  i_obj->_start_pos = pos;
  i_obj->_end_pos = pos - height * rot.col(0);
}

Cursor::~Cursor()
{
	delete i_obj;
	i_obj = NULL;
}

void Cursor::recomputeFromTransform(const Vector3d& pos, const Matrix3d& rot)
{
	i_obj->_start_pos = pos;
	i_obj->_end_pos = pos - height * rot.col(0);
}

void Cursor::draw()
{
	drawCylinder(i_obj->_start_pos, i_obj->_end_pos, i_obj->_radius, open?0.0:0.5, open?0.5:0.0, 0.0);
	drawSphere(i_obj->_start_pos, i_obj->_radius, open?0.0:0.5, open?0.5:0.0, 0.0);
	drawSphere(i_obj->_end_pos, i_obj->_radius, open?0.0:0.5, open?0.5:0.0, 0.0);
}

bool Cursor::capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections)
{
  Vector3d direction;
  double intersection_dist = capsuleCapsuleDistance(start, end, radius, i_obj->_start_pos, i_obj->_end_pos, i_obj->_radius, direction);
  if(intersection_dist < 0) {
    intersections.push_back(Intersection(capsule_ind, -intersection_dist, direction));
    return true;
  }
	return false;
}

double Cursor::capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius)
{
	if (REPULSION_COEFF <= 0.0) { return 0.0; }
	Vector3d direction;
	double dist = capsuleCapsuleDistance(start, end, radius, i_obj->_start_pos, i_obj->_end_pos, i_obj->_radius, direction);
	if (dist < 0 || dist > radius)
		return 0.0;
	return REPULSION_COEFF/2.0 * pow(dist-radius,2);
}

void Cursor::capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient)
{
	if (REPULSION_COEFF <= 0.0) { return; }
	Vector3d direction;
	double dist = capsuleCapsuleDistance(start, end, radius, i_obj->_start_pos, i_obj->_end_pos, i_obj->_radius, direction);
	if (dist < 0 || dist > radius)
		return;
	gradient -= REPULSION_COEFF * (radius - dist) * direction.normalized();
}

void Cursor::attach(EndEffector* ee)
{
	end_eff = ee;
	end_eff->attach(this);
}

// Dettaches the cursor from the end effector it is holding. It has to be holding an end effector. If the end effector isn't holding the thread, it is removed from the environment.
void Cursor::dettach()
{
	if (end_eff == NULL)
		cout << "Internal errror: Cursor::dettach(): cursor cannot dettach since it does't have an end effector attached" << endl;
	end_eff->dettach();
	if (end_eff->constraint<0)
		delete end_eff;
	end_eff = NULL;
}

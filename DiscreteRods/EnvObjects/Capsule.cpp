#include "Capsule.h"
#include "../threadpiece_discrete.h"

Capsule::Capsule(const Vector3d& pos, const Matrix3d& rot, double h, double r, float c0, float c1, float c2)
	: EnvObject(pos, rot, c0, c1, c2)
	, height(h)
	, radius(r)
{
	i_obj = new Intersection_Object();
	i_obj->_radius = r;
  i_obj->_start_pos = pos;
  i_obj->_end_pos = pos - h * rot.col(0);
}

Capsule::~Capsule()
{
	delete i_obj;
	i_obj = NULL;
}

void Capsule::recomputeFromTransform(const Vector3d& pos, const Matrix3d& rot)
{
	i_obj->_start_pos = pos;
	i_obj->_end_pos 	= pos - height * rot.col(0);
}

void Capsule::draw()
{
	drawCylinder(i_obj->_start_pos, i_obj->_end_pos, i_obj->_radius, color0, color1, color2);
	drawSphere(i_obj->_start_pos, i_obj->_radius, color0, color1, color2);
	drawSphere(i_obj->_end_pos, i_obj->_radius, color0, color1, color2);
}

bool Capsule::capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections)
{
  Vector3d direction;
  double intersection_dist = capsuleCapsuleDistance(start, end, radius, i_obj->_start_pos, i_obj->_end_pos, i_obj->_radius, direction);
  if(intersection_dist < 0) {
    intersections.push_back(Intersection(capsule_ind, -intersection_dist, direction));
    return true;
  }
	return false;
}

double Capsule::capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius)
{
	if (REPULSION_COEFF <= 0.0) { return 0.0; }
	Vector3d direction;
	double dist = capsuleCapsuleDistance(start, end, radius, i_obj->_start_pos, i_obj->_end_pos, i_obj->_radius, direction);
	if (dist < 0 || dist > radius)
		return 0.0;
	return REPULSION_COEFF/2.0 * pow(dist-radius,2);
}

void Capsule::capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient)
{
	if (REPULSION_COEFF <= 0.0) { return; }
	Vector3d direction;
	double dist = capsuleCapsuleDistance(start, end, radius, i_obj->_start_pos, i_obj->_end_pos, i_obj->_radius, direction);
	if (dist < 0 || dist > radius)
		return;
	gradient -= REPULSION_COEFF * (radius - dist) * direction.normalized();
}

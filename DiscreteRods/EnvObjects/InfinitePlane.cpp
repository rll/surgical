#include "InfinitePlane.h"
#include "../threadpiece_discrete.h"

InfinitePlane::InfinitePlane(const Vector3d& pos, const Vector3d& norm, float c0, float c1, float c2)
	: EnvObject(pos, Matrix3d::Identity(), c0, c1, c2)
	, normal(norm)
	, side(100.0)
{
	rotation_from_tangent(norm.normalized(), rotation);
}

InfinitePlane::~InfinitePlane() {}

void InfinitePlane::recomputeFromTransform(const Vector3d& pos, const Matrix3d& rot)
{
	normal = rot.col(0);
}

void InfinitePlane::draw()
{
	Vector3d x = rotation.col(1);
	Vector3d y = rotation.col(2);

	glPushMatrix();	
	glColor3f(color0, color1, color2);
	glDisable(GL_CULL_FACE);

	glBegin(GL_QUADS);
	glVertex3f(position(0) + side*(x(0)+y(0)), position(1) + side*(x(1)+y(1)), position(2) + side*(x(2)+y(2)));
	glVertex3f(position(0) + side*(x(0)-y(0)), position(1) + side*(x(1)-y(1)), position(2) + side*(x(2)-y(2)));
	glVertex3f(position(0) + side*(-x(0)-y(0)), position(1) + side*(-x(1)-y(1)), position(2) + side*(-x(2)-y(2)));
	glVertex3f(position(0) + side*(-x(0)+y(0)), position(1) + side*(-x(1)+y(1)), position(2) + side*(-x(2)+y(2)));
	glEnd();

	glEnable(GL_CULL_FACE);
	glPopMatrix();
}

bool InfinitePlane::capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections)
{
  Vector3d direction;
  double intersection_dist = capsuleInfinitePlaneDistance(start, end, radius, position, normal, direction);
  if(intersection_dist < 0) {
    intersections.push_back(Intersection(capsule_ind, -intersection_dist, direction));
    return true;
  }
	return false;
}

double InfinitePlane::capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius)
{
	if (REPULSION_COEFF <= 0.0) { return 0.0; }
	Vector3d direction;
	double dist = capsuleInfinitePlaneDistance(start, end, radius, position, normal, direction);
	if (dist < 0 || dist > radius)
		return 0.0;
	return REPULSION_COEFF/2.0 * pow(dist-radius,2);
}

void InfinitePlane::capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient)
{
	if (REPULSION_COEFF <= 0.0) { return; }
	Vector3d direction;
	double dist = capsuleInfinitePlaneDistance(start, end, radius, position, normal, direction);
	if (dist < 0 || dist > radius)
		return;
	//if (direction == Vector3d::Zero()) {
	//	cout << "dist: " << dist << endl;
	//	return;
	//}
	gradient -= REPULSION_COEFF * (radius - dist) * normal;
}

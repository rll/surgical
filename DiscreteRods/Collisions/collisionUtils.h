#ifndef _collisionUtils_h
#define _collisionUtils_h

#include <stdlib.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <btBulletDynamicsCommon.h>

#include "../threadutils_discrete.h"
#include "../threadpiece_discrete.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

class btCollisionShape;
class btCollisionDispatcher;
class btDefaultCollisionConfiguration;

inline double repulsionEnergyLog(const double& dist, const double& range, const Vector3d& direction)
{
	double a = range / 2.0;
	if (dist < 0) {
		return REPULSION_COEFF * 10000.0;
	} else if (dist < a) {
		return REPULSION_COEFF * (-log(dist/a) + 0.5);
	} else if (dist < 2.0*a) {
		return REPULSION_COEFF * (dist*dist/(a*a*2) - 2*dist/a + 2);
	} else {
		return 0.0;
	}
}

inline Vector3d repulsionEnergyGradientLog(const double& dist, const double& range, const Vector3d& direction)
{
	double a = range / 2.0;
	if (dist < 0) {
		return abs(dist) * direction.normalized();
	} else if (dist < a) {
		return REPULSION_COEFF * (1/dist) * direction.normalized();
	} else if (dist < 2.0*a) {
		return - REPULSION_COEFF * (dist/(a*a) - 2/a) * direction.normalized();
	} else {
		return Vector3d::Zero();
	}
}

inline double repulsionEnergyLinear(const double& dist, const double& range, const Vector3d& direction)
{
	if (dist < range) {
		//return REPULSION_COEFF/2.0 * pow(dist/range-1,2) * range;
		return REPULSION_COEFF * 0.5 * pow(dist-range,2);
	} else {
		return 0.0;
	}
}

inline Vector3d repulsionEnergyGradientLinear(const double& dist, const double& range, const Vector3d& direction)
{
	if (dist < range) {
		return REPULSION_COEFF * (range - dist) * direction.normalized();
	} else {
		return Vector3d::Zero();
	}
}

inline double repulsionEnergy(const double& dist, const double& range, const Vector3d& direction)
{
	return repulsionEnergyLog(dist, range, direction);
	//return repulsionEnergyLinear(dist, range, direction);
}

inline Vector3d repulsionEnergyGradient(const double& dist, const double& range, const Vector3d& direction)
{
	return repulsionEnergyGradientLog(dist, range, direction);
	//return repulsionEnergyGradientLinear(dist, range, direction);
}

//Returns the minimun distance between a capsule and a plane. If the distance is positive, the capsule is not colliding. If the distance is negative, the absolute value of it is the minimun intersecting distance.
//direction is a vector describing the minimun movement the capsule needs to take in order to fix the interesection.
double capsuleInfinitePlaneDistance(const Vector3d& a_start, const Vector3d& a_end, const double a_radius, const Vector3d& b_point, const Vector3d& b_normal, Vector3d& direction);

double capsuleBoxDistance(const Vector3d& a_start, const Vector3d& a_end, const double a_radius, const Vector3d& b_center, const Vector3d& b_half_length, Vector3d& direction);

double sphereBoxDistance(const Vector3d& a_center, const double a_radius, const Vector3d& b_center, const Vector3d& b_half_length, Vector3d& direction);

//Returns the minimun distance between capsules. If the distance is positive, the capsules are not colliding. If the distance is negative, the absolute value of it is the intersecting distance.
//direction is a vector pointing in the direction from capsule b to capsule a, and its norm is the minimun distance from its centers.
double capsuleCapsuleDistance(const Vector3d& a_start, const Vector3d& a_end, const double a_radius, const Vector3d& b_start, const Vector3d& b_end, const double b_radius, Vector3d& direction);

double capsuleSphereDistance(const Vector3d& a_start, const Vector3d& a_end, const double a_radius, const Vector3d& b_start, const double b_radius, Vector3d& direction);

#endif

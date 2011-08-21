#ifndef _collisionUtils_h
#define _collisionUtils_h

#include <stdlib.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

//Returns the minimun distance between a capsule and a plane. If the distance is positive, the capsule is not colliding. If the distance is negative, the absolute value of it is the minimun intersecting distance.
//direction is a vector describing the minimun movement the capsule needs to take in order to fix the interesection.
double capsuleInfinitePlaneDistance(const Vector3d& a_start, const Vector3d& a_end, const double a_radius, const Vector3d& b_point, const Vector3d& b_normal, Vector3d& direction);

//Returns the minimun distance between capsules. If the distance is positive, the capsules are not colliding. If the distance is negative, the absolute value of it is the intersecting distance.
//direction is a vector pointing in the direction from capsule b to capsule a, and its norm is the minimun distance from its centers.
double capsuleCapsuleDistance(const Vector3d& a_start, const Vector3d& a_end, const double a_radius, const Vector3d& b_start, const Vector3d& b_end, const double b_radius, Vector3d& direction);

#endif

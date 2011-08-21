#ifndef _InfinitePlane_h
#define _InfinitePlane_h

#include "EnvObject.h"

class InfinitePlane : public EnvObject
{
	public:
		InfinitePlane(const Vector3d& pos, const Vector3d& norm, float c0, float c1, float c2);
		~InfinitePlane();
		
		void recomputeFromTransform(const Vector3d& pos, const Matrix3d& rot);
		void draw();
		bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections);
  	double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius);
  	void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient);
	
	protected:
		Vector3d normal;
		double side;
};

#endif

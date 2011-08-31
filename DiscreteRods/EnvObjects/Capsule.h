#ifdef NEVERDEFINED
#ifndef _Capsule_h
#define _Capsule_h

#include "EnvObject.h"

class Capsule : public EnvObject
{
	public:
		Capsule(const Vector3d& pos, const Matrix3d& rot, double h, double r, float c0, float c1, float c2);
		~Capsule();
		
		// For saving and loading objects to and from files
		void writeToFile(ofstream& file);
		Capsule(ifstream& file);
		void updateIndFromPointers(World* world) {}
		void linkPointersFromInd(World* world) {}
		
		void recomputeFromTransform(const Vector3d& pos, const Matrix3d& rot);
		void draw();
		bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections);
  	double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius);
  	void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient);

	protected:
		Intersection_Object* i_obj;
		double height, radius;
};

#endif
#endif

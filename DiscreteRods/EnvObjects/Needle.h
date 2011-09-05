#ifndef _Needle_h
#define _Needle_h

#define MIN_ANGLE 0.1
#define MIN_RADIUS 0.1

#include "EnvObject.h"

class Needle : public EnvObject
{
	public:
		Needle(const Vector3d& pos, const Matrix3d& rot, double degrees, double r, float c0, float c1, float c2, World* w);
		Needle(const Vector3d& pos, const Vector3d& start_pos, const Vector3d& end_pos, float c0, float c1, float c2, World* w);
		Needle(const Vector3d& pos, const Vector3d& center_start, double degrees, const Vector3d& axis, float c0, float c1, float c2, World* w);
		Needle(const Needle& rhs, World* w);
		~Needle();
		
		//saving and loading from and to file
		void writeToFile(ofstream& file);
		Needle(ifstream& file, World* w);

		void setTransform(const Vector3d& pos, const Matrix3d& rot);
		Vector3d getStartPosition();
		Vector3d getEndPosition();
		double getAngle();
		double getRadius();
		Vector3d getAxis();
		void rotateAboutAxis(double degrees);
		
		void draw();
		
		//backup
		void backup();
		void restore();
		
		//collision
		bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections);
  	double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius);
  	void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient);
	
	protected:
		double angle;
		double radius;
		World* world;
		vector<Intersection_Object*> i_objs;

		//needs to be backup
		//position
		//rotation
		
		//needs to be restored
		//position
		//rotation
};

#endif

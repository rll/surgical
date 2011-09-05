#ifndef _Box_h
#define _Box_h

#include "EnvObject.h"

class Box : public EnvObject
{
	public:
		Box(const Vector3d& pos, const Matrix3d& rot, const Vector3d& half_length_xyz, float c0, float c1, float c2, World* w);
		Box(const Box& rhs, World* w);
		~Box();
		
		//saving and loading from and to file
		void writeToFile(ofstream& file);
		Box(ifstream& file, World* w);
		
		void setTransform(const Vector3d& pos, const Matrix3d& rot);
		
		void draw();
		
		//backup
		void backup();
		void restore();
		
		//collision
		bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections);
  	double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius);
  	void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient);
	
	protected:
		Vector3d half_length;
		World* world;

		//needs to be backup
		//position
		//rotation
		
		//needs to be restored
		//position
		//rotation
		
		// needs to be updated ONLY if the position or rotation changes
		vector<Vector3d> normals;
		vector<vector<Vector3d> > vertex_positions;
};

#endif

#ifndef _Box_h
#define _Box_h

#include "EnvObject.h"

class ThreadConstrained;
class Needle;

class Box : public EnvObject
{
	friend class Cursor;
	friend class EndEffector;
	
	public:
		Box(const Vector3d& pos, const Matrix3d& rot, const Vector3d& half_length_xyz, float c0, float c1, float c2, World* w, Needle* n = NULL, ThreadConstrained* t = NULL, int constrained_vertex_num0 = -1, int constrained_vertex_num1 = -1);
		Box(const Box& rhs, World* w);
		~Box();
		
		//saving and loading from and to file
		void writeToFile(ofstream& file);
		Box(ifstream& file, World* w);
		
		void setTransform(const Vector3d& pos, const Matrix3d& rot);
		
		void draw();
		
		const Vector3d& getHalfLength() const { return half_length; }
		void insertNeedle(Needle* n);
		void stepThread();
		
		//thread attachment
		void attach(ThreadConstrained* t) { thread = t; }
		void dettachThread() { thread = NULL; }
		bool isThreadAttached() { return (thread!=NULL); }
		ThreadConstrained* getThread() { return thread; }
		//needle attachment
		void attach(Needle* n) { needle = n; }
		void dettachNeedle() { needle = NULL; }
		bool isNeedleAttached() { return (needle!=NULL); }
		Needle* getNeedle() { return needle; }
		
		//backup
		void backup();
		void restore();
		
		//collision
		bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections);
  	double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius);
  	void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient);
		
	protected:
		Vector3d half_length;
		ThreadConstrained* thread;
		int constraint0;
		int constraint1;
		Vector3d c0_pos;
		Matrix3d c0_rot;
		Vector3d c1_pos;
		Matrix3d c1_rot;
		Needle* needle;
		World* world;

		// needs to be updated ONLY if the position or rotation changes
		vector<Vector3d> normals;
		vector<vector<Vector3d> > vertex_positions;
		
		//backup
		int backup_constraint0;
		int backup_constraint1;
		int backup_thread_ind;				// -1 if there is no thread inside the box
		int backup_needle_ind;
		
		//needs to be backup
		//position
		//rotation
		//constraint0
		//constraint1
		//thread_ind (backup_thread_ind = world->objectIndex<ThreadConstrained>(thread)) This is equivalent to backing up the thread pointer
		//needle_ind (backup_needle_ind = world->objectIndex<Needle>(needle)) This is equivalent to backing up the needle pointer
		
		//needs to be restored
		//position
		//rotation
		//constraint0
		//constraint1
		//thread (thread = world->ObjectAtIndex<ThreadConstrained>(backup_thread_ind);
		//needle (needle = world->objectAtIndex<Needle>(backup_needle_ind);
};

#endif

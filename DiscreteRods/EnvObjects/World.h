#ifndef _World_h
#define _World_h

#include "EnvObject.h"

class ThreadConstrained;

class World
{
	public:
		World();
		~World();
		
		void addThread(ThreadConstrained* thread);
		void addEnvObj(EnvObject* obj);
		vector<EnvObject*>* getEnvObjs();
		vector<ThreadConstrained*>* getThreads();
		void clearObjs();
		
		void draw();
		bool capsuleObjectIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections);
		double capsuleObjectRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius);
		void capsuleObjectRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient);
	
	private:
		vector<ThreadConstrained*> threads;
		vector<EnvObject*> objs;
};

#endif

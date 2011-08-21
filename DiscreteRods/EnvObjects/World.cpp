#include "World.h"
#include "../ThreadConstrained.h"

World::World()
{}

World::~World()
{
	clearObjs();
}

void World::addThread(ThreadConstrained* thread)
{
	threads.push_back(thread);
	thread->setWorld(this);
}

void World::addEnvObj(EnvObject* obj) {	objs.push_back(obj); }

vector<EnvObject*>* World::getEnvObjs()
{
	return &objs;
}

vector<ThreadConstrained*>* World::getThreads()
{
	return &threads;
}

void World::clearObjs()
{
	for (int i = 0; i<threads.size(); i++) {
		delete threads[i];
		threads[i] = NULL;
	}
	for (int i = 0; i<objs.size(); i++) {
		delete objs[i];
		objs[i] = NULL;
	}
	threads.clear();
	objs.clear();
}

void World::draw()
{
	for (int i = 0; i<threads.size(); i++)
		threads[i]->draw();
	for (int i = 0; i<objs.size(); i++)
		objs[i]->draw();
}

bool World::capsuleObjectIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections)
{
	bool found = false;
	for (int obj_ind = 0; obj_ind < objs.size(); obj_ind++) {
		found = objs[obj_ind]->capsuleIntersection(capsule_ind, start, end, radius, intersections) || found;
	}
	return found;
}

double World::capsuleObjectRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius)
{
	double energy = 0.0;
	for (int obj_ind = 0; obj_ind < objs.size(); obj_ind++) {
		energy += objs[obj_ind]->capsuleRepulsionEnergy(start, end, radius);
	}
	return energy;
}

void World::capsuleObjectRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient)
{
	for (int obj_ind = 0; obj_ind < objs.size(); obj_ind++) {
		objs[obj_ind]->capsuleRepulsionEnergyGradient(start, end, radius, gradient);
	}
}

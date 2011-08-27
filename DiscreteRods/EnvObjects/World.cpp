#include "World.h"
#include "EnvObject.h"
#include "Capsule.h"
#include "Cursor.h"
#include "EndEffector.h"
#include "InfinitePlane.h"
#include "TexturedSphere.h"
#include "../ThreadConstrained.h"
#include "../thread_discrete.h"
#include "../IO/ControlBase.h"

World::World()
{}

//TODO handle NO_OBJECT case and other resmaining cases
World::World(const World& rhs)
{
	threads.resize(rhs.threads.size());
	for (int i = 0; i<rhs.threads.size(); i++) {
		threads[i] = new ThreadConstrained(*(rhs.threads[i]));
	}
	objs.resize(rhs.objs.size());
	for (int i = 0; i<rhs.objs.size(); i++) {
		rhs.objs[i]->updateIndFromPointers((World*) &rhs);
		switch (rhs.objs[i]->getType())
    {
      case CURSOR:
        {
          objs[i] = new Cursor(*(dynamic_cast<Cursor*>(rhs.objs[i])));
          addEnvObj(objs[i]);
          break;
        }
      case END_EFFECTOR:
        {
          objs[i] = new EndEffector(*(dynamic_cast<EndEffector*>(rhs.objs[i])));
          addEnvObj(objs[i]);
          break;
        }
      case INFINITE_PLANE:
        {
          objs[i] = new InfinitePlane(*(dynamic_cast<InfinitePlane*>(rhs.objs[i])));
          addEnvObj(objs[i]);
          break;
        }
      case TEXTURED_SPHERE:
        {
          objs[i] = new TexturedSphere(*(dynamic_cast<TexturedSphere*>(rhs.objs[i])));
          addEnvObj(objs[i]);
          break;
        }
    }
  }
  
  initializeThreadsInEnvironment();
  
  for (int i = 0; i<objs.size(); i++) {
  	objs[i]->linkPointersFromInd(this);
  }
}   
/*
//TODO old elements should be deleted
World& World::operator=(const World& rhs)
{
	threads.resize(rhs.threads.size());
	for (int i = 0; i<rhs.threads.size(); i++) {
		threads[i] = new ThreadConstrained(*(rhs.threads[i]));
	}
	objs.resize(rhs.objs.size());
	for (int i = 0; i<rhs.objs.size(); i++) {
		switch (rhs.objs[i]->getType())
    {
      case CURSOR:
        {
          objs[i] = new Cursor(*(dynamic_cast<Cursor*>(rhs.objs[i])));
          addEnvObj(objs[i]);
          break;
        }
      case END_EFFECTOR:
        {
          objs[i] = new EndEffector(*(dynamic_cast<EndEffector*>(rhs.objs[i])));
          addEnvObj(objs[i]);
          break;
        }
      case INFINITE_PLANE:
        {
          objs[i] = new InfinitePlane(*(dynamic_cast<InfinitePlane*>(rhs.objs[i])));
          addEnvObj(objs[i]);
          break;
        }
      case TEXTURED_SPHERE:
        {
          objs[i] = new TexturedSphere(*(dynamic_cast<TexturedSphere*>(rhs.objs[i])));
          addEnvObj(objs[i]);
          break;
        }
    }
  }
  
  return *this;
}
*/

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

vector<EnvObject*> World::getEnvObjs(object_type type)
{
	vector<EnvObject*> objs_of_type;
	for (int i=0; i<objs.size(); i++) {
		if (objs[i]->getType() == type)
			objs_of_type.push_back(objs[i]);
	}
	return objs_of_type;
}

void World::getEnvObjs(vector<EnvObject*>& objects, object_type type)
{
	objects.clear();
	for (int i=0; i<objs.size(); i++) {
		if (objs[i]->getType() == type)
			objects.push_back(objs[i]);
	}
}

/*EnvObject* World::getEnvObj(int ind, object_type type)
{	
	assert(0); //TODO verify it works;
	for (int i=0; i<objs.size(); i++) {
		if (objs[i]->getType() == type) {
			if (ind == 0)
			 	return objs[i];
			else if
				ind--;
		}
	}
	assert(0); //ind was out of bounds for that particulat type of object
}*/

vector<ThreadConstrained*>* World::getThreads()
{
	return &threads;
}

// Updates the threads_in_env variable of every Thread object in the world (i.e. every Thread of every ThreadConstrained in the world). This variable is used for thread-thread collisions.
void World::initializeThreadsInEnvironment() {
	vector<ThreadConstrained*>* all_thread_constrained = this->getThreads();
	vector<Thread*> all_threads;
	for (int k=0; k<(*all_thread_constrained).size(); k++) {
		ThreadConstrained* thread_constrained = (*all_thread_constrained)[k];
		vector<Thread*>* threads = thread_constrained->getThreads();
		for (int i=0; i<(*threads).size(); i++)
			all_threads.push_back((*threads)[i]);
	}
	for (int i=0; i<all_threads.size(); i++) {
		all_threads[i]->clear_threads_in_env();
		for (int j=0; j<all_threads.size(); j++) {
			if (i!=j) 
				all_threads[i]->add_thread_to_env(all_threads[j]);
		}
	}
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

void World::applyControl(const vector<ControlBase*>& controls)
{
	vector<EnvObject*> cursors = getEnvObjs(CURSOR);
	assert(cursors.size() == controls.size());
	for (int i = 0; i < cursors.size(); i++) {
		Cursor* cursor = (dynamic_cast<Cursor*>(cursors[i]));
		cursor->setTransform(controls[i]->getPosition(), controls[i]->getRotation());
		if (controls[i]->hasButtonPressedAndReset(UP))
			cursor->openClose();
		if (controls[i]->hasButtonPressedAndReset(DOWN))
			cursor->attachDettach();
	}
	
	vector<EnvObject*> ee_env_objs;
	getEnvObjs(ee_env_objs, END_EFFECTOR);
	vector<EndEffector*> end_effectors;
	for (int i=0; i<ee_env_objs.size(); i++) {
		end_effectors.push_back(dynamic_cast<EndEffector*>(ee_env_objs[i]));
	}
	
	for (int thread_ind = 0; thread_ind < threads.size(); thread_ind++) {
		
		vector<EndEffector*> thread_end_effs;
		for (int ee_ind = 0; ee_ind < end_effectors.size(); ee_ind++)
			if (end_effectors[ee_ind]->getThread()==threads[thread_ind])
				thread_end_effs.push_back(end_effectors[ee_ind]);

		vector<Vector3d> positionConstraints;
		vector<Matrix3d> rotationConstraints;
		threads[thread_ind]->getConstrainedTransforms(positionConstraints, rotationConstraints);

		for (int ee_ind = 0; ee_ind < thread_end_effs.size(); ee_ind++) {
			EndEffector* ee = thread_end_effs[ee_ind];
			positionConstraints[ee->constraint_ind] = ee->getPosition();
			rotationConstraints[ee->constraint_ind] = ee->getRotation();
		}

		threads[thread_ind]->updateConstraints(positionConstraints, rotationConstraints);
		threads[thread_ind]->getConstrainedTransforms(positionConstraints, rotationConstraints);
		
		for (int ee_ind = 0; ee_ind < thread_end_effs.size(); ee_ind++) {
			EndEffector* ee = thread_end_effs[ee_ind];
			ee->forceSetTransform(positionConstraints[ee->constraint_ind], rotationConstraints[ee->constraint_ind]);
		}

		//threads[thread_ind]->adapt_links();
	}
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

void World::getStates(vector<VectorXd>& states)
{
	states.clear();
	for (int i = 0; i<threads.size(); i++) {
		VectorXd state;
		threads[i]->getState(state);
		states.push_back(state);
	}
	for (int i = 0; i<objs.size(); i++) {
		VectorXd state;
		objs[i]->getState(state);
		states.push_back(state);
	}
}

void World::saveToBackup()
{
	for (int i = 0; i < threads.size(); i++) {
		threads[i]->saveToBackup();
	}
	for (int i = 0; i<objs.size(); i++) {
		objs[i]->saveToBackup();
	}
}

void World::restoreFromBackup()
{
	for (int i = 0; i < threads.size(); i++) {
		threads[i]->restoreFromBackup();
	}
	for (int i = 0; i<objs.size(); i++) {
		objs[i]->restoreFromBackup();
	}
}


#ifndef _World_h
#define _World_h

#include <stdlib.h>

#ifdef MAC
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <GL/gle.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h>
#endif

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>

#include "../utils/drawUtils.h"
#include "../Collisions/intersectionStructs.h"
#include "../Collisions/collisionUtils.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

enum object_type {NO_OBJECT, THREAD_CONSTRAINED, CAPSULE, END_EFFECTOR, INFINITE_PLANE, TEXTURED_SPHERE};

class EnvObject;

class Cursor;

class EndEffector;

class ControlBase;

class ThreadConstrained;

class World
{
	public:
		World();
		World(const World& rhs);
    World& operator=(const World& rhs);
		~World();

		//manipulate threads and objects in environment
		//void addThread(ThreadConstrained* thread);
		//void addEnvObj(EnvObject* obj);
		
		void getEnvObjs(vector<EnvObject*>& objects);
		void getEnvObjs(vector<EnvObject*>& objects, object_type type);
		void getThreads(vector<ThreadConstrained*>& ths);
		int threadIndex(ThreadConstrained* thread);
		ThreadConstrained* threadAtIndex(int thread_ind);
		int cursorIndex(Cursor* cursor);
		Cursor* cursorAtIndex(int cursor_ind);
		int envObjIndex(EnvObject* env_obj);
		EnvObject* envObjAtIndex(int env_obj_ind);
				
		void clearObjs();

		void initializeThreadsInEnvironment();
		EndEffector* closestEndEffector(Vector3d tip_pos);
	
		void draw();

		//applying control
		//the controls should know to whom they are applying control.
		//if the control doesn't have an ee attachment, world should solve that; i.e. find the closest ee for the control.
		//cursors are used as a handle for controls and the objects in the world
		// for each control, there is 3 dof for translation, 3 for rotation, 2 for event
		void applyControl(const vector<ControlBase*>& controls); //applies controli to handlei
		void applyRelativeControl(const VectorXd& relative_control);
		void setThreadConstraintsFromEndEffs();
		
		void getStates(vector<VectorXd>& states);
		
		//backup
		void backup();
		void restore();
		
		//collision
		bool capsuleObjectIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections);
		double capsuleObjectRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius);
		void capsuleObjectRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient);
		
		//Init thread
		void initThread();
		void initLongerThread();
		void initRestingThread();
		
	private:
		vector<ThreadConstrained*> threads;
		vector<EnvObject*> objs;
		vector<Cursor*> cursors; //control handler
};

#endif

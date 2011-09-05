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

enum object_type {NO_OBJECT, THREAD_CONSTRAINED, CAPSULE, END_EFFECTOR, INFINITE_PLANE, TEXTURED_SPHERE, CURSOR};

class EnvObject;

class Cursor;

class EndEffector;

class ControllerBase;

class Control;

class ThreadConstrained;

class World
{
	public:
		World();
		World(const World& rhs);
		~World();

		//saving and loading from and to file
		void writeToFile(ofstream& file);
		World(ifstream& file);

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
	
		void draw(bool examine_mode = false);

		//applying control
		//the controls should know to whom they are applying control.
		//if the control doesn't have an ee attachment, world should solve that; i.e. find the closest ee for the control.
		//cursors are used as a handle for controls and the objects in the world
		// for each control, there is 3 dof for translation, 3 for rotation, 2 for event
		void setTransformFromController(const vector<ControllerBase*>& controls, bool limit_displacement = false); //applies controli to handlei
		void applyRelativeControl(const vector<Control*>& controls, bool limit_displacement = false);
		void applyRelativeControl(const VectorXd& relative_control, bool limit_displacement = false);
		void setThreadConstraintsFromEndEffs();
    void applyRelativeControlJacobian(const VectorXd& relative_control); 
		
		//state representation
    void getStates(vector<VectorXd>& states);
    void printStates();

    // Jacobian
    void getStateForJacobian(VectorXd& world_state);
    void setStateForJacobian(VectorXd& world_state);
    void projectLegalState();
    void computeJacobian(MatrixXd& J); 
    
		
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
		vector<Cursor*> cursors; //control handler
		vector<ThreadConstrained*> threads;
		vector<EnvObject*> objs;
};

#endif

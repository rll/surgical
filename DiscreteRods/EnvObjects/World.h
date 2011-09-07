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
#include <typeinfo>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>

#include "../utils/drawUtils.h"
#include "../Collisions/intersectionStructs.h"
#include "../Collisions/collisionUtils.h"

#include "ObjectTypes.h"
#include "EnvObject.h"
#include "Capsule.h"
#include "Cursor.h"
#include "EndEffector.h"
#include "InfinitePlane.h"
#include "TexturedSphere.h"
#include "Box.h"
#include "Needle.h"

#include "../IO/Control.h"
#include "../IO/ControllerBase.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

#ifdef NDEBUG
	#define TYPE_CAST static_cast
#else
	#define TYPE_CAST dynamic_cast
#endif

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

		template <class T> void getObjects(vector<T*>& objects)
		{
			objects.clear();
			if (typeid(ThreadConstrained) == typeid(T)) {
				for (int i=0; i<threads.size(); i++) {
					objects.push_back(reinterpret_cast<T*>(threads[i]));
				}
			} else if (typeid(Cursor) == typeid(T)) {
				for (int i=0; i<cursors.size(); i++) {
					objects.push_back(reinterpret_cast<T*>(cursors[i]));
				}
			} else {
				for (int i=0; i<objs.size(); i++) {
					if (typeid(*objs[i]) == typeid(T))
						objects.push_back(dynamic_cast<T*>(objs[i]));
				}
			} 
		}
		
		template <class T> int objectIndex(T* object)
		{
			vector<T*> objects;
			getObjects<T>(objects);
			int i;
			for (i=0; i<objects.size() && objects[i]!=object; i++) {}
			if (i == objects.size())
				return -1;	
			return i;
		}
		
		template <class T> T* objectAtIndex(int object_ind)
		{
			vector<T*> objects;
			getObjects<T>(objects);
			assert((object_ind >= -1) && (object_ind < (int)objects.size()));
			if (object_ind == -1)
				return NULL;
			return objects[object_ind];
		}
				
		void clearObjs();

		void initializeThreadsInEnvironment();
		EndEffector* closestEndEffector(Vector3d tip_pos);
	
		void draw(bool examine_mode = false);
		void drawDebug();

		//applying control
		//the controls should know to whom they are applying control.
		//if the control doesn't have an ee attachment, world should solve that; i.e. find the closest ee for the control.
		//cursors are used as a handle for controls and the objects in the world
		// for each control, there is 3 dof for translation, 3 for rotation, 2 for event
		void setTransformFromController(const vector<ControllerBase*>& controls, bool limit_displacement = false); //applies controli to handlei
		void applyRelativeControl(const vector<Control*>& controls, bool limit_displacement = false);
		void applyRelativeControl(const VectorXd& relative_control, bool limit_displacement = false);
    void applyRelativeControlJacobian(const VectorXd& relative_control); 
		
		//state representation
    void getStates(vector<VectorXd>& states);
    void printStates();

    // Jacobian
    void getStateForJacobian(VectorXd& world_state); 
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
		void initRestingThread(int opt);
		
	private:
		vector<Cursor*> cursors; //control handler
		vector<ThreadConstrained*> threads;
		vector<EnvObject*> objs;
};

#endif

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
#include <boost/thread.hpp>
#include <boost/random.hpp>

#include "../utils/drawUtils.h"
#include "../Collisions/intersectionStructs.h"
#include "../Collisions/CollisionWorld.h"
#include "WorldManager.h"

#include "ObjectTypes.h"
#include "Object.h"
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

extern WorldManager* test_world_manager;

enum RenderMode { NORMAL, EXAMINE, DEBUG, COLLISION };

class ThreadConstrained;

extern WorldManager* test_world_manager;

class World
{
	public:
		World(WorldManager* wm = NULL);
		World(const World& rhs, WorldManager* wm = NULL);
		~World();

		//saving and loading from and to file
		void writeToFile(ofstream& file);
		World(ifstream& file, WorldManager* wm = NULL);

		template <class T> void getObjects(vector<T*>& objects) const
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
		
		template <class T> int objectIndex(T* object) const
		{
			vector<T*> objects;
			getObjects<T>(objects);
			int i;
			for (i=0; i<objects.size() && objects[i]!=object; i++) {}
			if (i == objects.size())
				return -1;	
			return i;
		}
		
		template <class T> T* objectAtIndex(int object_ind) const
		{
			vector<T*> objects;
			getObjects<T>(objects);
			assert((object_ind >= -1) && (object_ind < (int)objects.size()));
			if (object_ind == -1)
				return NULL;
			return objects[object_ind];
		}
				
		EndEffector* closestEndEffector(Vector3d tip_pos);
	
		void draw(RenderMode examine_mode = NORMAL);

		double distanceMetric(const World* w);
		void updateTransformsFromThread();
		
		//applying control
		//the controls should know to whom they are applying control.
		//if the control doesn't have an ee attachment, world should solve that; i.e. find the closest ee for the control.
		//cursors are used as a handle for controls and the objects in the world
		// for each control, there is 3 dof for translation, 3 for rotation, 2 for event
		void setTransformFromController(const vector<ControllerBase*>& controls, bool limit_displacement = false); //applies controli to handle
		void applyRelativeControl(const vector<Control*>& controls, double thresh=0.0, bool limit_displacement = false);
		void applyRelativeControl(const vector<Control*>& controls, vector<double>& displacements, double thresh=0.0, bool limit_displacement = false);
		void applyRelativeControl(const VectorXd& relative_control, double thresh=0.0, bool limit_displacement = false);
    void applyRelativeControlJacobian(const VectorXd& relative_control, double thresh=0.0);
		
		//state representation
    void getStates(vector<VectorXd>& states);
    void printStates();

    // Jacobian
    void getStateForJacobian(VectorXd& world_state);
    void setStateForJacobian(VectorXd& world_state);
    void projectLegalState();
    void computeJacobian(MatrixXd* J); 

    //control representation
    void VectorXdToControl(const VectorXd& relative_control, vector<Control*>& c) {
      assert(cursors.size()*8 == relative_control.size());

      c.resize(cursors.size());

      for (int i = 0; i < cursors.size(); i++) { 
        Vector3d translation = relative_control.segment(8*i, 3);
        Matrix3d rotation;
        rotation_from_euler_angles(rotation, relative_control(8*i+3), relative_control(8*i+4), relative_control(8*i+5));

        Control* u = new Control();

        u->setTranslate(translation);
        u->setRotate(rotation);
        c[i] = u;
      }
    }

    void ControlToVectorXd(const vector<Control*>& c, VectorXd& relative_control) {
      assert(cursors.size() == c.size());

      relative_control.resize(cursors.size()*8);
      relative_control.setZero();
      cout << relative_control.size() << endl;
      for (int i = 0; i < cursors.size(); i++) { 
        Vector3d translation = c[i]->getTranslate();
        Matrix3d rotate(c[i]->getRotate());
        relative_control.segment(i*8, 3) = translation; 
        euler_angles_from_rotation(rotate, relative_control(i*8+3),
            relative_control(i*8+4), relative_control(i*8+5));

      }
    }

    VectorXd JacobianControlStripper(const VectorXd& relative_control) {
      assert(cursors.size()*8 == relative_control.size());
      VectorXd stripped_control(12);
      stripped_control.segment(0,6) = relative_control.segment(0,6);
      stripped_control.segment(6,6) = relative_control.segment(8,6);
      return stripped_control;
    }

    VectorXd JacobianControlWrapper(const VectorXd& relative_control) { 

      assert(cursors.size()*6 == relative_control.size());
      VectorXd wrapper_control(16);
      wrapper_control.setZero(); 
      wrapper_control.segment(0, 6) = relative_control.segment(0,6);
      wrapper_control.segment(8, 6) = relative_control.segment(6,6);
      return wrapper_control;
    }
    
		//backup
		void backup();
		void restore();
		
		//collision
		CollisionWorld* collision_world;
		WorldManager* world_manager;
				
		//Init thread
		void initThread();
		void initThreadSingle();
		void initLongerThread();
		void initRestingThread(int opt);
		void initRestingFinerThread(int opt);
		
	private:
		vector<Cursor*> cursors; //control handler
		vector<ThreadConstrained*> threads;
		vector<EnvObject*> objs;
};

void computeJacCord(World* w, int i, int size_each_state, VectorXd *du, MatrixXd *J);


#endif

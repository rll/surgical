#ifndef _CollisionWorld_h
#define _CollisionWorld_h

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

#include <math.h>

#include <btBulletDynamicsCommon.h>
#include "../../utils/drawUtils.h"

#include "../EnvObjects/ObjectTypes.h"
#include "../EnvObjects/Object.h"

using namespace std;

class CollisionWorld
{
	public:
		CollisionWorld();
		~CollisionWorld();
		
		void addCollisionObject(btCollisionObject* obj);
		void removeCollisionObject(btCollisionObject* obj);
		
		static bool needCollision(btCollisionObject* obA, btCollisionObject* obB);
		struct FilterCallback : public btOverlapFilterCallback
		{
   		// return true when pairs need collision
   		virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const;
   	};
		//void nearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo);
		void performDiscreteCollisionDetection();
		double totalRepulsionEnergy(Thread* thread);
		void totalRepulsionEnergyGradient(Thread* thread, vector<Vector3d>& vertex_gradient);
		void drawAllCollisions();
		void announceAllCollisions();
		
		btCollisionWorld* collision_world;		
		btOverlapFilterCallback * filter_callback;
};

#endif //CollisionWorld

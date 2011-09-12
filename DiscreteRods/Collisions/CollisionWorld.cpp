#include "CollisionWorld.h"
#include "../thread_discrete.h"

CollisionWorld::CollisionWorld()
{
	//bullet physics collision detection
	//btDefaultCollisionConfiguration* collision_configuration = new btDefaultCollisionConfiguration();
	
	btDefaultCollisionConstructionInfo constructionInfo = btDefaultCollisionConstructionInfo();
	constructionInfo.m_defaultMaxCollisionAlgorithmPoolSize = 512;	//default
	constructionInfo.m_defaultMaxPersistentManifoldPoolSize = 512;	//default
	
	btDefaultCollisionConfiguration* collision_configuration = new btDefaultCollisionConfiguration(constructionInfo);

	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collision_configuration);
//	btVector3	world_aabb_min(-100,-100,-100);
//	btVector3	world_aabb_max(100,100,100);

//	btAxisSweep3*	broadphase = new btAxisSweep3(world_aabb_min,world_aabb_max);
	
	btDbvtBroadphase* broadphase = new btDbvtBroadphase();
	
	//SimpleBroadphase is a brute force alternative, performing N^2 aabb overlap tests
	//SimpleBroadphase*	broadphase = new btSimpleBroadphase;

	collision_world = new btCollisionWorld(dispatcher,broadphase,collision_configuration);

	filter_callback = new FilterCallback();
}

CollisionWorld::~CollisionWorld()
{
	delete collision_world->getBroadphase();
	delete collision_world->getDispatcher();
	delete filter_callback;
	filter_callback = NULL;
	delete collision_world;
	collision_world = NULL;
}

void CollisionWorld::addCollisionObject(btCollisionObject* obj)
{
	collision_world->addCollisionObject(obj);
}

void CollisionWorld::removeCollisionObject(btCollisionObject* obj)
{
	collision_world->removeCollisionObject(obj);
}

bool CollisionWorld::needCollision(btCollisionObject* obA, btCollisionObject* obB)
{
	bool collides = true;
	Object* objA = static_cast<Object*>(obA->getUserPointer());
  Object* objB = static_cast<Object*>(obB->getUserPointer());
  if (objA->getType() == objB->getType()) {
  	if(objA->getType() == THREAD_PIECE) {
  		ThreadPiece* thread_pieceA = dynamic_cast<ThreadPiece*>(objA);
  		ThreadPiece* thread_pieceB = dynamic_cast<ThreadPiece*>(objB);
  		
  		if (thread_pieceA->_my_thread == thread_pieceB->_my_thread) {
  			collides = collides && (abs(thread_pieceA->_piece_ind - thread_pieceB->_piece_ind) > 1);
  		}
  		
  		collides = collides && !(((thread_pieceA->_piece_ind == 0) || (thread_pieceA->_piece_ind == thread_pieceA->_my_thread->_thread_pieces.size()-2)) &&
  																																																((thread_pieceB->_piece_ind == 0) || (thread_pieceB->_piece_ind == thread_pieceB->_my_thread->_thread_pieces.size()-2)));

  	} else if (objA->getType() == END_EFFECTOR) {
  		collides = collides && (objA != objB);
  	}
  } else if ((objA->getType() == THREAD_PIECE) && (objB->getType() == END_EFFECTOR)) {
  	ThreadPiece* thread_pieceA = dynamic_cast<ThreadPiece*>(objA);
  	collides = collides && (thread_pieceA->_piece_ind > 1) && (thread_pieceA->_piece_ind < thread_pieceA->_my_thread->_thread_pieces.size()-3);      	
  } else if ((objA->getType() == END_EFFECTOR) && (objB->getType() == THREAD_PIECE)) {
  	ThreadPiece* thread_pieceB = dynamic_cast<ThreadPiece*>(objB);
  	collides = collides && (thread_pieceB->_piece_ind > 1) && (thread_pieceB->_piece_ind < thread_pieceB->_my_thread->_thread_pieces.size()-3);
  } else {
  	assert(0);
  }
  return collides;
}

// return true when pairs need collision
bool CollisionWorld::FilterCallback::needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
{
  bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
  collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
  
  btCollisionObject* obA = static_cast<btCollisionObject*>(proxy0->m_clientObject);
	btCollisionObject* obB = static_cast<btCollisionObject*>(proxy1->m_clientObject);
	      
  //add some additional logic here that modified 'collides'
  collides = collides && needCollision(obA, obB);
  
  return collides;
}

void nearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo)
{
	// Do your collision logic here
  // Only dispatch the Bullet collision information if you want the physics to continue
  btCollisionObject* obA = static_cast<btCollisionObject*>(collisionPair.m_pProxy0->m_clientObject);
	btCollisionObject* obB = static_cast<btCollisionObject*>(collisionPair.m_pProxy1->m_clientObject);
	
	if (CollisionWorld::needCollision(obA, obB))
    dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
}

//this should be called whenever any object is moved
void CollisionWorld::performDiscreteCollisionDetection()
{
	btVector3	world_bounds_min,world_bounds_max;
	collision_world->getBroadphase()->getBroadphaseAabb(world_bounds_min,world_bounds_max);
	
	collision_world->getBroadphase()->getOverlappingPairCache()->setOverlapFilterCallback(filter_callback);
	
	dynamic_cast<btCollisionDispatcher*>(collision_world->getDispatcher())->setNearCallback(nearCallback);
	
	if (collision_world)
		collision_world->performDiscreteCollisionDetection();
}

double CollisionWorld::totalRepulsionEnergy(Thread* thread)
{
	double energy = 0;
	
	int numManifolds = collision_world->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++)
	{
		btPersistentManifold* contact_manifold = collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contact_manifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contact_manifold->getBody1());
		
		Object* objA = static_cast<Object*>(obA->getUserPointer());
    Object* objB = static_cast<Object*>(obB->getUserPointer());
    if ((objA->getType() == THREAD_PIECE) || (objB->getType() == THREAD_PIECE)) {
  		int num_contacts = contact_manifold->getNumContacts();
			for (int j=0;j<num_contacts;j++) {
  			btManifoldPoint& pt = contact_manifold->getContactPoint(j);
  			if (pt.getDistance()<0.f)	{
  				const double dist = pt.getDistance() + 2.0*REPULSION_DIST;
  				if (objA->getType() == THREAD_PIECE) {
  					ThreadPiece* thread_pieceA = dynamic_cast<ThreadPiece*>(objA);
  					if (thread_pieceA->_my_thread == thread) {
  						energy += repulsionEnergy(dist, 2.0*REPULSION_DIST);
						}
					}
  				if (objB->getType() == THREAD_PIECE) {
  					ThreadPiece* thread_pieceB = dynamic_cast<ThreadPiece*>(objB);
  					if (thread_pieceB->_my_thread == thread) {
	  					energy += repulsionEnergy(dist, 2.0*REPULSION_DIST);
						}
  				}
  			}
  		}
  	}
  }
  return energy;
}

void CollisionWorld::totalRepulsionEnergyGradient(Thread* thread, vector<Vector3d>& vertex_gradients)
{
	int numManifolds = collision_world->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++)
	{
		btPersistentManifold* contact_manifold = collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contact_manifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contact_manifold->getBody1());
		
		Object* objA = static_cast<Object*>(obA->getUserPointer());
    Object* objB = static_cast<Object*>(obB->getUserPointer());
    if ((objA->getType() == THREAD_PIECE) || (objB->getType() == THREAD_PIECE)) {
  		int num_contacts = contact_manifold->getNumContacts();
			for (int j=0;j<num_contacts;j++) {
  			btManifoldPoint& pt = contact_manifold->getContactPoint(j);
  			if (pt.getDistance()<0.f)	{
  				const double dist = pt.getDistance() + 2.0*REPULSION_DIST;
  				const Vector3d direction = toVector3d(pt.getPositionWorldOnA()) - toVector3d(pt.getPositionWorldOnB());
  				const Vector3d grad = repulsionEnergyGradient(dist, 2.0*REPULSION_DIST, direction);
  				if (objA->getType() == THREAD_PIECE) {
  					ThreadPiece* thread_pieceA = dynamic_cast<ThreadPiece*>(objA);
  					if (thread_pieceA->_my_thread == thread) {
  						vertex_gradients[thread_pieceA->_piece_ind] += grad;
							vertex_gradients[thread_pieceA->_piece_ind+1] += grad;
						}
					}
  				if (objB->getType() == THREAD_PIECE) {
  					ThreadPiece* thread_pieceB = dynamic_cast<ThreadPiece*>(objB);
  					if (thread_pieceB->_my_thread == thread) {
	  					vertex_gradients[thread_pieceB->_piece_ind] -= grad;
							vertex_gradients[thread_pieceB->_piece_ind+1] -= grad;
						}
  				}
  			}
  		}
  	}
  }
}

//Assume performDiscreteCollisionDetection has been called
void CollisionWorld::drawAllCollisions()
{
	int numManifolds = collision_world->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++)
	{
		btPersistentManifold* contact_manifold = collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contact_manifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contact_manifold->getBody1());
		
		int num_contacts = contact_manifold->getNumContacts();
		for (int j=0;j<num_contacts;j++)
		{
			btManifoldPoint& pt = contact_manifold->getContactPoint(j);

			if (pt.getDistance()<0.f)
			{
				glDisable(GL_LIGHTING);
				glBegin(GL_LINES);
				glColor3f(0.8, 0, 0);
			
				btVector3 ptA = pt.getPositionWorldOnA();
				btVector3 ptB = pt.getPositionWorldOnB();

				glVertex3d(ptA.x(),ptA.y(),ptA.z());
				glVertex3d(ptB.x(),ptB.y(),ptB.z());
				glEnd();
				glEnable(GL_LIGHTING);
			}
		}

		//you can un-comment out this line, and then all points are removed
		//contactManifold->clearManifold();	
	}
}

//Assume performDiscreteCollisionDetection has been called
void CollisionWorld::announceAllCollisions()
{
	int numManifolds = collision_world->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++)
	{
		btPersistentManifold* contact_manifold = collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contact_manifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contact_manifold->getBody1());
		
		int num_contacts = contact_manifold->getNumContacts();
		for (int j=0;j<num_contacts;j++)
		{
			btManifoldPoint& pt = contact_manifold->getContactPoint(j);

			if (pt.getDistance()<0.f)
			{
				if (pt.getDistance() < -2.0*REPULSION_DIST)
					cout << "penetration of " << -pt.getDistance()-2.0*REPULSION_DIST << endl;
			}
		}

		//you can un-comment out this line, and then all points are removed
		//contactManifold->clearManifold();	
	}
}

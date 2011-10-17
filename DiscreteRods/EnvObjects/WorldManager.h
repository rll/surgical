#ifndef _WorldManager_h
#define _WorldManager_h

#include <stdlib.h>
#include <vector>
#include <boost/thread.hpp>

#include <btBulletDynamicsCommon.h>

class World;
class CollisionWorld;

using namespace std;

class WorldManager
{
	public:
		WorldManager();
		~WorldManager();

		CollisionWorld* allocateWorld(World* world);
		void freeWorld(World* world);
		int numOfAllocatedWorlds();
		int numOfAllocatedCollisionWorlds();

	protected:
		vector<World*> worlds;
		vector<CollisionWorld*> collision_worlds;

  private:
    boost::mutex mymutex; 
};

#endif //WorldManager

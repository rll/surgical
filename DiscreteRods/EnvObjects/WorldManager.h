#ifndef _WorldManager_h
#define _WorldManager_h

#include <stdlib.h>
#include <vector>

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
};

#endif //WorldManager

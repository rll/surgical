#include "WorldManager.h"
#include "World.h"
#include "../Collisions/CollisionWorld.h"

WorldManager::WorldManager()
{

}

WorldManager::~WorldManager()
{
	for (int i = 0; i < worlds.size(); i++) {
		worlds[i] = NULL;
	}
	for (int i = 0; i < collision_worlds.size(); i++) {
		assert(collision_worlds[i] != NULL);
		delete collision_worlds[i];
		collision_worlds[i] = NULL;
	}
	worlds.clear();
	collision_worlds.clear();
}

CollisionWorld* WorldManager::allocateWorld(World* world)
{
	assert(worlds.size() == collision_worlds.size());
	int world_ind;
	for (world_ind = 0; (world_ind < worlds.size()) && (worlds[world_ind] != NULL); world_ind++) {}
	if (world_ind == worlds.size()) {
		worlds.push_back(world);
		CollisionWorld* collision_world = new CollisionWorld();
		collision_worlds.push_back(collision_world);
		return collision_world;
	} else {
		worlds[world_ind] = world;
		assert(collision_worlds[world_ind] != NULL);
#ifndef NDEBUG
		//make sure collision_worlds don't have any btCollisionObjects in it. i.e. the destructor for World should had removed all the collision objects from the world.
		btCollisionObjectArray col_objs = collision_worlds[world_ind]->collision_world->getCollisionObjectArray();
		assert(col_objs.size() == 0);
#endif
		return collision_worlds[world_ind];
	}
}

void WorldManager::freeWorld(World* world)
{
	assert(worlds.size() == collision_worlds.size());
	int world_ind;
	for (world_ind = 0; (world_ind < worlds.size()) && (worlds[world_ind] != world); world_ind++) {}
	assert(world_ind != worlds.size());
	worlds[world_ind] = NULL;
}

int WorldManager::numOfAllocatedWorlds()
{
	assert(worlds.size() == collision_worlds.size());
	int num_worlds = 0;
	for (int world_ind = 0; world_ind < worlds.size(); world_ind++) {
		if (worlds[world_ind] != NULL)
			num_worlds++;
	}
	return num_worlds;
}

int WorldManager::numOfAllocatedCollisionWorlds()
{
	assert(worlds.size() == collision_worlds.size());
	return collision_worlds.size();
}

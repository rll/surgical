#ifndef _RRTNode_h
#define _RRTNode_h

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <algorithm>
#include <vector>

#include "rrt_utils.h"
#include "../DiscreteRods/EnvObjects/World.h"

USING_PART_OF_NAMESPACE_EIGEN

enum RRTDrawMode { NO_DRAW, START, END, START_AND_END };

class RRTNode
{
  public:
    RRTNode(World* w, RRTNode* p = NULL);
    ~RRTNode();

    void planTrajectory(RRTNode*& goal_node, World const * goal);

    double distanceMetric(RRTNode* node);

    /**
     * sample_world should not be empty. This function takes the sample_world,
     * and modifies the state of each of it's threads randomly.
     */
    void sampleRandomWorld(World* sample_world);

		void sampleWorldFromWorld(World* sample_world, World* world);

    /**
     * Nearest_node is set to the node whose world is nearest to sample_world. 
     * The nearest node can be this node or any of its children (recursively).
     */
    double nearestNeighbor(RRTNode*& nearest_node, World const * sample_world);

    /**
     * Extend from current node to a new extended_node whose world is close to
     * sample_thread. The new node is added to this node's children.
     */
    void extend(RRTNode*& extended_node, World const * sample_world);

    void branchTrajectory(vector<World*>& trajectory);

    void drawTree(RRTDrawMode mode = START_AND_END);
    void drawBranch();

  private:
    void reverseBranchTrajectory(vector<World*>& trajectory);    
    void drawTreeSpheres(RRTDrawMode mode);
		Vector3d drawTreeLines(RRTDrawMode mode);

    World* world;
    RRTNode* parent;
    vector<RRTNode*> children;  
};

#endif //RRTNode

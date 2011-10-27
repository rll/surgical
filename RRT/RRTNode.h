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

class RRTNode
{
  public:
    RRTNode(World* w, RRTNode* p = NULL);
    ~RRTNode();

    void planTrajectory(RRTNode*& goal_node, const World* goal);

    double distanceMetric(RRTNode* node);

    /**
     * sample_world should not be empty. This function takes the sample_world,
     * and modifies the state of each of it's threads randomly.
     */
    void sampleWorld(World* sample_world);

    /**
     * Nearest_node is set to the node whose world is nearest to sample_world. 
     * The nearest node can be this node or any of its children (recursively).
     */
    double nearestNeighbor(RRTNode*& nearest_node, const World* sample_world);

    /**
     * Extend from current node to a new extended_node whose world is close to
     * sample_thread. The new node is added to this node's children.
     */
    void extend(RRTNode*& extended_node, const World* sample_world);

    void branchTrajectory(vector<World*>& trajectory);

    Vector3d drawTree();
    void drawBranch();

  private:
    void reverseBranchTrajectory(vector<World*>& trajectory);

    World* world;
    RRTNode* parent;
    vector<RRTNode*> children;  
};

#endif //RRTNode

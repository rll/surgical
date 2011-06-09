#ifndef _thread_RRT_h
#define _thread_RRT_h

#include "thread_minenergy.h"
#include "thread_utils.h"
#include <Eigen/Geometry> 
#include <vector>


#define MAX_NUM_NODES 600
#define GOAL_DISTANCE_THRESH 3.0

#define RANDOM_SAMPLE_THRESH (double)0.75
#define ADD_TO_GOAL_DIST_RADIUS (double)8.0
#define ADD_TO_GOAL_TAN_RADIUS (double)0.4

//#define NUM_RANDOM_SAMPLES 5
//#define DISTANCE_TO_TRY 0.1
//#define ANG_TO_TRY M_PI/100.0
//#define MAX_DISTANCE_MOVE_EACH_DIR 5.0
#define MAX_ANG_TO_ROTATE M_PI/30.0
#define MAX_NOISE_ROTATION_VECTOR 0.2
#define MAX_NOISE_ROTATION_ANGLE M_PI/40.0

#define MAX_DISTANCE_MOVEMENT 5.0
#define MAX_NOISE_DISTANCE_MOVEMENT 3.0

#define NUM_PTS_PER_NODE 33


struct RRT_Node
{

	Thread* thread;
	MatrixXd points;

	RRT_Node(): thread(NULL){};
	RRT_Node(const RRT_Node& nodeToCopy);
	RRT_Node(const Thread* threadIn);
	~RRT_Node();
	void setPoints();
	void copyNode(const RRT_Node& toCopy);
	
	double distanceToPoints(MatrixXd& otherPoints);
	double distanceToNode(RRT_Node& otherNode);

	void applyMotion(RRT_Node& end, Thread_Motion& motion);

	RRT_Node& operator=(const RRT_Node& rhs);

};

struct RRT_Node_And_Edge
{
	RRT_Node this_node;
	RRT_Node_And_Edge* last_node;
	Thread_Motion* motion_from_last;

	RRT_Node_And_Edge():
		last_node(NULL), motion_from_last(NULL){};

	RRT_Node_And_Edge(RRT_Node& node) :
		this_node(node), last_node(NULL), motion_from_last(NULL){};

	RRT_Node_And_Edge(RRT_Node_And_Edge* last_node_in, Thread_Motion* motion_from_last_in) :
		last_node(last_node_in), motion_from_last(motion_from_last_in){};

	RRT_Node_And_Edge(RRT_Node& this_node_in, RRT_Node_And_Edge* last_node_in, Thread_Motion* motion_from_last_in) :
		this_node(this_node_in), last_node(last_node_in), motion_from_last(motion_from_last_in){};

	
	RRT_Node_And_Edge(const RRT_Node_And_Edge& toCopy) :
		this_node(toCopy.this_node), last_node(toCopy.last_node), motion_from_last(toCopy.motion_from_last) {};

	RRT_Node_And_Edge(const RRT_Node_And_Edge* toCopy) :
		this_node(toCopy->this_node), last_node(toCopy->last_node), motion_from_last(toCopy->motion_from_last) {};

	~RRT_Node_And_Edge();



	RRT_Node_And_Edge& operator=(const RRT_Node_And_Edge& rhs);


};


class Thread_RRT
{
	public:
		Thread_RRT();
		~Thread_RRT();

		void planPath(const Thread* start, const Thread* goal, vector<Thread_Motion>& movements, vector<Thread*>& intermediateThread);


	private:
		void initializeSearch(RRT_Node& start, RRT_Node& goal);	
		void getNextGoal(RRT_Node& goal, RRT_Node& goalThisIter); //samples either randomly, or around goal
		void setThisGoalNearGoal(RRT_Node& goal, RRT_Node& goalThisIter); //if we need to sample around goal, do so
		double findClosestNode(RRT_Node& goalNode, int& ind_to_closest);
		void findThreadMotion(RRT_Node& start, RRT_Node& goal, Thread_Motion& bestMotion);
		void cleanup();
		//void addToNodes(RRT_Node& toAdd);

		vector <RRT_Node_And_Edge*> currNodes;

};

#endif

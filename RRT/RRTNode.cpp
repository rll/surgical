#include "RRTNode.h"

RRTNode::RRTNode(World* w, RRTNode* p)
	: world(w)
	, parent(p)
{
	assert(w != NULL);
}

RRTNode::~RRTNode()
{
	//I delete the world of the children rather this this node's world because this node's world didn't create it's own world but instead the world of each of the children.
	for (int i = 0; i < children.size(); i++) {
		if (children[i] != NULL) {
			if (children[i]->world != NULL) {
				delete children[i]->world;
			}
			delete children[i];
		}
	}
	children.clear();
}

void RRTNode::planTrajectory(RRTNode*& goal_node, const World* goal)
{
	World* sample_world = new World(*world, world->world_manager);
	RRTNode *nearest_node = NULL, *extended_node = NULL;
	int iter = 0;
	do {
		sampleWorld(sample_world);
		nearestNeighbor(nearest_node, sample_world);
		nearest_node->extend(extended_node, sample_world);
		cout << "distance between extended_node->world and goal " << extended_node->world->distanceMetric(goal) << endl;
		iter++;
	} while (extended_node->world->distanceMetric(goal) > 50.0 && iter < 50);
	goal_node = extended_node;
	nearestNeighbor(goal_node, goal);
	
	delete sample_world;
}

void RRTNode::sampleWorld(World* sample_world)
{
	assert(sample_world != NULL);
	vector<ThreadConstrained*> threads;
	sample_world->getObjects<ThreadConstrained>(threads);

	for (int i = 0; i < threads.size(); i++) {
		generateRandomThread(threads[i]);
	}

	//Usually, the cursor's transform is modified, and this change is automatically propagated to the cursor's end effector (if any) and the end effectors thread (if any). In this case, we're modifying the thread end constraints, so we need to do special work to update the transform of the end effector and the cursor.
	vector<EndEffector*> end_effs;
	sample_world->getObjects<EndEffector>(end_effs);
	for (int ee_ind = 0; ee_ind < end_effs.size(); ee_ind++) {
		end_effs[ee_ind]->updateTransformFromAttachment(false);
	}
	
	vector<Cursor*> cursors;
	sample_world->getObjects<Cursor>(cursors);
	for (int i = 0; i < cursors.size(); i++) {
		cursors[i]->updateTransformFromEndEffector();
	}
}

double RRTNode::distanceMetric(RRTNode* node)
{
	return world->distanceMetric(node->world);
}

double RRTNode::nearestNeighbor(RRTNode*& nearest_node, const World* sample_world)
{
	nearest_node = this;
	double dist = this->world->distanceMetric(sample_world);
	for (int i = 0; i < children.size(); i++) {
		RRTNode* candidate_node;
		double candidate_dist = children[i]->nearestNeighbor(candidate_node, sample_world);
		if (candidate_dist < dist) {
			dist = candidate_dist;
			nearest_node = candidate_node;
		}
	}
	return dist;
}

void RRTNode::extend(RRTNode*& extended_node, const World* sample_world)
{
	World* extended_world = new World(*world, world->world_manager);
	double sample_extended_dist = extended_world->distanceMetric(sample_world);
	World* candidate_world = NULL;
	double sample_candidate_dist;
	
	vector<Control*> controls;
	Vector3d translation;
	Quaterniond rotation;
	
	for (int trial = 0; trial < 10; trial++) {
		for (int control_ind = 0; control_ind < 2; control_ind++) {
			uniformlyRandomTranslation(translation, 20.0);
			//TODO Assume rotation at the ends remain constant. If not a random rotation should be generated
			//uniformlyRandomRotation(rotation);
			rotation.setIdentity();
			controls.push_back(new Control(translation, rotation));
		}
		
		candidate_world = new World(*world, world->world_manager);
		candidate_world->applyRelativeControl(controls);
		sample_candidate_dist = candidate_world->distanceMetric(sample_world);
		
		if (sample_candidate_dist < sample_extended_dist) {
			delete extended_world;
			extended_world = candidate_world;
			sample_extended_dist = sample_candidate_dist;
		} else {
			delete candidate_world;
			candidate_world = NULL;
		}
		
		for (int control_ind = 0; control_ind < 2; control_ind++) {
			delete controls[control_ind];
			controls[control_ind] = NULL;
		}
		controls.clear();
	}

	extended_node = new RRTNode(extended_world, this);
	children.push_back(extended_node);
}

void RRTNode::branchTrajectory(vector<World*>& trajectory)
{
	trajectory.clear();
	reverseBranchTrajectory(trajectory);
	reverse(trajectory.begin(), trajectory.end());
}

void RRTNode::reverseBranchTrajectory(vector<World*>& trajectory)
{
	trajectory.push_back(world);
	if (parent != NULL)
		parent->reverseBranchTrajectory(trajectory);
}

void RRTNode::drawTree()
{
	world->draw();
	for (int i = 0; i < children.size(); i++) {
		children[i]->drawTree();
	}
}

void RRTNode::drawBranch()
{
	world->draw();
	if (parent != NULL)
		parent->drawBranch();
}






#include "RRTNode.h"
#include "simple_rrt.h"

RRTNode::RRTNode(World* w, RRTNode* p)
  : world(w)
  , parent(p)
{
  assert(w != NULL);
}

RRTNode::~RRTNode()
{
  // I delete the world of the children rather this this node's world because
  // this node's world didn't create it's own world but instead the world of 
  // each of the children.
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

void RRTNode::planTrajectory(RRTNode*& goal_node, World const * goal)
{
  World* sample_world = new World(*world, world->world_manager);
  RRTNode *nearest_node = NULL, *extended_node = NULL;
  // TODO the following while loop is limited by the number of iterations since
  // the taken samples don't converge fast enough
  int iter = 0;
  int max_iter = 500;
  do {
    if (drand48() < 0.9) { // exploration
	    sampleRandomWorld(sample_world);
	 	} else { // kind of exploitation
	 		sampleWorldFromWorld(sample_world, (World*) goal);
	 	}
    nearestNeighbor(nearest_node, sample_world);
    nearest_node->extend(extended_node, sample_world);
    cout << "iter " << iter << "/" << max_iter-1 << ". distance between extended_node->world and goal " << extended_node->world->distanceMetric(goal) << endl;
    iter++;
    drawFromView(extended_node->world->draw);
  } while (extended_node->world->distanceMetric(goal) > 10.0 && iter < max_iter);
  goal_node = extended_node;
  if (iter == max_iter)
    nearestNeighbor(goal_node, goal);
  cout << "Finished with a distance of " << extended_node->world->distanceMetric(goal) << endl;
  
  delete sample_world;
}

void RRTNode::sampleRandomWorld(World* sample_world)
{
  assert(sample_world != NULL);
  vector<ThreadConstrained*> threads;
  sample_world->getObjects<ThreadConstrained>(threads);

  for (int i = 0; i < threads.size(); i++) {
    generateRandomThread(threads[i]);
  }

  // Usually, the cursor's transform is modified, and this change is 
  // automatically propagated to the cursor's end effector (if any) and the 
  // end effectors thread (if any). In this case, we're modifying the thread 
  // end constraints, so we need to do special work to update the transform of 
  // the end effector and the cursor.
  sample_world->updateTransformsFromThread();
}

void RRTNode::sampleWorldFromWorld(World* sample_world, World* world)
{
  assert(sample_world != NULL);
  vector<ThreadConstrained*> sample_threads;
  sample_world->getObjects<ThreadConstrained>(sample_threads);
  assert(world != NULL);
  vector<ThreadConstrained*> threads;
  world->getObjects<ThreadConstrained>(threads);
	assert(sample_threads.size() == threads.size());

  for (int i = 0; i < sample_threads.size(); i++) {
    sample_threads[i]->copyData(*threads[i]);
  }

  sample_world->updateTransformsFromThread();
}

double RRTNode::distanceMetric2(RRTNode* node)
{
	VectorXd world_state;
  world->getStateForJacobian(world_state);
  
  VectorXd node_world_state;
  node->world->getStateForJacobian(node_world_state);
  
  return (world_state - node_world_state).norm();
}

// Assumes the world and node->world has only one thread.
double RRTNode::distanceMetric(RRTNode* node)
{
	Vector3d start_position;
	Vector3d end_position;
	vector<Matrix3d> material_frames;
  world->objectAtIndex<ThreadConstrained>(0)->get_thread_data(start_position, end_position, material_frames);
	
	Vector3d node_start_position;
	Vector3d node_end_position;
	vector<Matrix3d> node_material_frames;
  node->world->objectAtIndex<ThreadConstrained>(0)->get_thread_data(node_start_position, node_end_position, node_material_frames);
  
  VectorXd diff;
  diff.resize(6+3*(material_frames.size()-1));
  diff.segment(0, 3) = (node_start_position - start_position);
  diff.segment(3, 3) =  (node_end_position - end_position);
  for (int i = 0; i < (material_frames.size()-1); i++) {
  	diff.segment(6+3*i, 3) = (node_material_frames[i].col(0) - material_frames[i].col(0));
  }
  
  return diff.norm();
}

double RRTNode::nearestNeighbor(RRTNode*& nearest_node, World const * sample_world)
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

void RRTNode::extend(RRTNode*& extended_node, World const * sample_world)
{
  vector<Control*> controls;
  Vector3d translation;
  Quaterniond rotation;
  
  World* extended_world = NULL;
  double sample_extended_dist = DBL_MAX;
  
  /*
  // Create a Control such that its translation and rotation would move the Cursor it controls from world's Cursor transform to sample_world's Cursor transform.
  for (int control_ind = 0; control_ind < 2; control_ind++) {
    translation = sample_world->objectAtIndex<Cursor>(control_ind)->getPosition() - world->objectAtIndex<Cursor>(control_ind)->getPosition();
    rotation = world->objectAtIndex<Cursor>(control_ind)->getRotation().transpose() * sample_world->objectAtIndex<Cursor>(control_ind)->getRotation();
    controls.push_back(new Control(translation, rotation));
  }
  
  World* extended_world = new World(*world, world->world_manager);
  // The 'true' argument in the applyRelativeControl specifies that the translation and rotation should be interpolated (i.e. limited).
  extended_world->applyRelativeControl(controls, 0.0, true);
  double sample_extended_dist = extended_world->distanceMetric(sample_world);
  
  for (int control_ind = 0; control_ind < 2; control_ind++) {
    delete controls[control_ind];
    controls[control_ind] = NULL;
  }
  controls.clear();
  */
  
  World* candidate_world = NULL;
  double sample_candidate_dist;

  // Now create Controls with random translation and rotation, and keep the extended_world that is closest to the sample_world.
  for (int trial = 0; trial < 10; trial++) {
    for (int control_ind = 0; control_ind < 2; control_ind++) {
      uniformlyRandomTranslation(translation, 5.0);
      uniformlyRandomRotation(rotation);
      controls.push_back(new Control(translation, rotation));
    }

    candidate_world = new World(*world, world->world_manager);
    // The 'true' argument in the applyRelativeControl specifies that the translation and rotation should be interpolated (i.e. limited).
    candidate_world->applyRelativeControl(controls, 0.0, true, 10.0*MAX_DISPLACEMENT, 50.0*MAX_ANGLE_CHANGE);
    sample_candidate_dist = candidate_world->distanceMetric(sample_world);
    
    if (extended_world == NULL) {
      extended_world = candidate_world;
      sample_extended_dist = sample_candidate_dist;
    } else if (sample_candidate_dist < sample_extended_dist) {
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

void RRTNode::drawTree(RRTDrawMode mode)
{
  if (mode == NO_DRAW)
  	return;
  glColor3f(1.0, 1.0, 1.0);
  glPushMatrix();
  if (mode == START_AND_END) {
  	drawTreeSpheres(START);
  	drawTreeSpheres(END);
  } else {
  	drawTreeSpheres(mode);
  }
  glBegin(GL_LINES);
  glLineWidth(10.0);
  if (mode == START_AND_END) {
  	drawTreeLines(START);
  	drawTreeLines(END);
  } else {
  	drawTreeLines(mode);
  }
  glEnd();
  glPopMatrix();
}

void RRTNode::drawTreeSpheres(RRTDrawMode mode)
{
	Vector3d pos;
	if (mode == START)
		pos = world->objectAtIndex<ThreadConstrained>(0)->start_pos();
	else
		pos = world->objectAtIndex<ThreadConstrained>(0)->end_pos();
  drawSphere(pos, 0.5);
	for (int i = 0; i < children.size(); i++) {
    children[i]->drawTreeSpheres(mode);
  }
}

Vector3d RRTNode::drawTreeLines(RRTDrawMode mode)
{
	Vector3d pos;
	if (mode == START)
		pos = world->objectAtIndex<ThreadConstrained>(0)->start_pos();
	else
		pos = world->objectAtIndex<ThreadConstrained>(0)->end_pos();
	for (int i = 0; i < children.size(); i++) {
    glVertex3f(pos);
    glVertex3f(children[i]->drawTreeLines(mode));
  }
  return pos;
}

void RRTNode::drawBranch()
{
  world->draw();
  if (parent != NULL)
    parent->drawBranch();
}




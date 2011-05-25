#include "planner_utils.h"
#include <time.h>
#include <omp.h>


omp_lock_t writelock; 

Thread_RRT::Thread_RRT()
{
}

Thread_RRT::~Thread_RRT()
{
  for (int i = 0; i < _tree.size(); i++) { 
    delete _tree[i]; 
  }
  _tree.clear(); 
  

}

bool Thread_RRT::hasnan(Thread* tocheck)
{
  vector<Vector3d> points;
  vector<double> twist_angles;
  tocheck->get_thread_data(points, twist_angles);

  //write each point for each thread
  for (int i=0; i < points.size(); i++)
  {
    if (isnan(points[i].norm()))
      return true;
  }

  return false;
}


void Thread_RRT::initialize(const Thread* start, const Thread* goal)
{

  omp_init_lock(&writelock);
  Thread* start_copy = new Thread(*start);
  Thread* goal_copy = new Thread(*goal); 
  _start_node = new RRTNode(start_copy);
  _goal_node = new RRTNode(goal_copy);  

  for (int i=0; i < _tree.size(); i++)
  {
    delete _tree[i];
  }
  _tree.clear();

  distToGoal = DBL_MAX;
  bestDist = DBL_MAX;
  TOLERANCE = 0.01;
  totalPenalty = 1.0; 

  index = new LshMultiTable<HASH>();
  HASH::Parameter *param = new HASH::Parameter();
  unsigned int dim = start->num_pieces()*3;
  param->dim = dim;
  param->repeat = 16;

  index->init(*param, 2); 

  insertIntoRRT((RRTNode *) _start_node); 


}
void Thread_RRT::planStep(Thread& new_sample_thread, Thread& closest_sample_thread, Thread& new_extend_thread) { 
  //cout << "getting target" << endl;
 // Vector3d startEdge = target->vertex_at_ind(1) - target->vertex_at_ind(0);
  Thread* next_target = NULL; 
  double newSampleDist = DBL_MAX;
  while (newSampleDist == DBL_MAX) { 
    next_target = getNextGoal();
    newSampleDist = extendAsFarToward(next_target);
    if (newSampleDist == DBL_MAX) {
      delete next_target;
    }
  }

  omp_set_lock(&writelock);
  new_sample_thread = *next_target;
  closest_sample_thread = *(_tree.back()->prev->thread);
  new_extend_thread = *(_tree.back()->thread);
  omp_unset_lock(&writelock);
  //cout << "done step" << endl;

}

void Thread_RRT::updateBestPath() {
  RRTNode* closest = findClosestNode(_goal_node->thread, false);
  while(closest->prev != NULL) {
    closest->prev->next = closest;
    closest = closest->prev;
  }
}


Thread* Thread_RRT::doubleDimApproximation(const Thread* target) {
  vector<Vector3d> vertices;
  vector<double> angles;


  Vector3d startEdge = target->start_edge();
 // Vector3d startEdge = target->vertex_at_ind(1) - target->vertex_at_ind(0);
  Vector3d endEdge = target->end_edge();
  startEdge.normalize();
  endEdge.normalize();

  vertices.push_back(target->vertex_at_ind(1)-0.5*startEdge*target->rest_length());
  angles.push_back(0.0);
  vertices.push_back(target->vertex_at_ind(1));
  //vertices.push_back(target->start_pos()+startEdge*target->rest_length());
  angles.push_back(0.0);
  

  for (int i = 2; i < target->num_pieces()-1; i++) {
    vertices.push_back((target->vertex_at_ind(i)+target->vertex_at_ind(i-1))/2);
    vertices.push_back(target->vertex_at_ind(i));
    angles.push_back(0.0);
    angles.push_back(0.0);
  }
  vertices.push_back(vertices[vertices.size()-1] + 0.5*endEdge*target->rest_length());

  angles.push_back(0.0);

  Vector3d target_start_pos = target->start_pos();
  Matrix3d target_start_rot = target->start_rot();
  Vector3d target_end_pos = target->end_pos();
  Matrix3d target_end_rot = target->end_rot(); 

  Thread* increasedDimThread = new Thread(vertices, angles, target_start_rot, 0.5*target->rest_length());
  //double rest_length_ratio = ((double)target->num_pieces()) / increasedDimThread->num_pieces();

  //double rest_length_ratio = 0.5;
  //increasedDimThread->set_rest_length(target->rest_length() * rest_length_ratio);
  increasedDimThread->set_end_twist(target->end_angle());
  increasedDimThread->set_end_constraint(vertices[vertices.size()-1], target_end_rot);
  increasedDimThread->project_length_constraint();
  

//  cout << increasedDimThread->num_pieces() << endl; 

  return increasedDimThread;

}

Thread* Thread_RRT::halfDimApproximation(const Thread* target) {

//  cout << "Start with p: " << target->num_pieces() << endl;

  vector<Vector3d> vertices;     
  vector<double> angles;

  Vector3d startEdge = target->start_edge();
  Vector3d endEdge = target->end_edge();
  

  startEdge.normalize();
  endEdge.normalize();

  vertices.push_back(target->vertex_at_ind(1) - 2*startEdge*target->rest_length());
  angles.push_back(0.0);
  vertices.push_back(target->vertex_at_ind(1));
  angles.push_back(0.0);

  for(int i = 2; i < target->num_pieces()-1; i++) {
    if ((i-1) % 2 == 0) { 
      vertices.push_back(target->vertex_at_ind(i));
      angles.push_back(0.0); 
    }
  }

  vertices.push_back(vertices[vertices.size()-1] + 2*endEdge*target->rest_length());
  angles.push_back(0.0);

  Matrix3d target_start_rot = target->start_rot();
  
  Thread* reducedDimensionThread = new Thread(vertices, angles, 
      target_start_rot, target->rest_length() * 2.0);
//  double rest_length_ratio = ((double)target->num_pieces()) / reducedDimensionThread->num_pieces();
  //double rest_length_ratio = 2.0; 
  //reducedDimensionThread->set_rest_length(target->rest_length()*rest_length_ratio);

  Vector3d target_end_pos = target->end_pos();
  Matrix3d target_end_rot = target->end_rot();

  reducedDimensionThread->set_end_twist(target->end_angle());
  reducedDimensionThread->set_end_constraint(vertices[vertices.size()-1], target_end_rot);
  reducedDimensionThread->project_length_constraint();
  //reducedDimensionThread->minimize_energy(20000, 1e-6, 0.2, 1e-7);
  //reducedDimensionThread->minimize_energy();


 // cout << reducedDimensionThread->num_pieces() << endl; 

  return reducedDimensionThread;

}


/*void Thread_RRT::planPath(const Thread* start, const Thread* goal, vector<Two_Motions>& movements) {
  initialize(start, goal);

  while(!(distToGoal < TOLERANCE)) {
    getNextGoal(&next, &next_rot);
    distToGoal = extendToward(next, next_rot);
    if(distToGoal < bestDist) {
      bestDist = distToGoal;
    }
    cout << "curr ind" << _tree.size() << " curr dist: " << distToGoal << " best dist: " << bestDist << endl;
  }

  // extract the solution movements
  cout << "Tree size: " << _tree.size() << endl;
  movements.clear();
  for(RRTNode* cur = _tree.back(); cur->prev != NULL; cur = cur->prev) {
    if(cur->prev == NULL) break;
    for(int i = 0; i < cur->lstMotions.size(); i++) {
      movements.push_back(*(cur->lstMotions[i]));
    }
  }
  reverse(movements.begin(), movements.end());
}*/


Thread* Thread_RRT::generateSample(const Thread* goal_thread) { 
  // links are sampled within a 45 degree sample with each other

  vector<Vector3d> vertices;
  vector<double> angles;
  int N = goal_thread->num_pieces();
  
  Vector3d start; 
  Vector3d goal; 
  
  double max_thread_length = N*goal_thread->rest_length(); 
  double noise_multiplier; 
  if (drand48() < 0.20) { 
     
    // sample the start by taking the current start and adding noise
    // sample the goal by taking the current goal and adding noise 
    //cout << "sampling near the start" << endl;
    
    noise_multiplier = goal_thread->rest_length(); 

  } else { 
    // sample the start by sampling in the sphere of the norm
    // these samples tend to increase the branching of the tree
    //cout << "sampling in the ball" << endl;
  
    noise_multiplier = max_thread_length / 5; // not exactly as big as we might like
  
  }


  start = goal_thread->start_pos();
  goal = goal_thread->end_pos(); 
  Vector3d noise_start;
  Vector3d noise_goal; 
  do {
    noise_start = (noise_start << Normal(0,1)*noise_multiplier,
        Normal(0,1)*noise_multiplier,
        Normal(0,1)*noise_multiplier).finished();
    noise_goal = (noise_goal << Normal(0,1)*noise_multiplier,
        Normal(0,1)*noise_multiplier,
        Normal(0,1)*noise_multiplier).finished();

  } while(((goal+noise_goal)-(start+noise_start)).norm() > max_thread_length/1.5);

  start += noise_start;
  goal += noise_goal;
  
  
  vertices.push_back(start);
  angles.push_back(0.0);

  double angle;
  Vector3d inc;
  
  do {
    inc << Normal(0,1), Normal(0,1), Normal(0,1);
    //inc = goal;
    inc.normalize();
    inc *= goal_thread->rest_length();
    angle = acos(inc.dot(goal)/(goal.norm()*inc.norm()));
  } while(abs(angle) > 3*M_PI/4.0);


  for(int i = 0; i < N-1; i++) {
    Vector3d prevInc = inc;
    vertices.push_back(vertices[vertices.size()-1] + inc);
    angles.push_back(0.0);
    // time to move toward the goal
    if ((vertices[vertices.size()-1] - goal).squaredNorm() > (N-2-i-1)*(N-2-i-1)*goal_thread->rest_length()*goal_thread->rest_length()) {
      inc = (goal - vertices[vertices.size()-1]).normalized()*goal_thread->rest_length();
      //if ( acos(inc.dot(prevInc) / (prevInc.norm() * inc.norm())) > 3*M_PI/4 ) {
        //inc = prevInc; 
      //}
    } else {
      if (drand48() < 0.25) {
        if (drand48() < 0.5) {
         do {
            inc << Normal(0,1), Normal(0,1), Normal(0,1);
            //inc = goal;
            inc.normalize();
            inc *= goal_thread->rest_length();
            angle = acos(inc.dot(goal) / (goal.norm()*inc.norm()));     
          } while(abs(angle) > M_PI/3.0);
        } else {
         do {
            inc << Normal(0,1), Normal(0,1), Normal(0,1);
            //inc = goal;
            inc.normalize();
            inc *= goal_thread->rest_length();
            angle = acos(inc.dot(prevInc) / (prevInc.norm()*inc.norm()));     
          } while(abs(angle) > M_PI/3.0);
        }
          //cout << angle << endl;
      }
    }
  }


  Matrix3d start_rot;
  //start_rot.setZero();
	start_rot.col(0) = (vertices[1]-vertices[0]).normalized();
	Vector3d col_1 = Vector3d::UnitY();
	make_vectors_perpendicular(start_rot.col(0), col_1);
	col_1.normalize();
	start_rot.col(1) = col_1;
	start_rot.col(2) = start_rot.col(0).cross(start_rot.col(1));
  //inc << Normal(0,1), Normal(0,1), Normal(0,1);
  //rotation_from_euler_angles(start_rot, inc(0), inc(1), inc(2));

 // Matrix3d end_rot;
//  end_rot.setZero();
//  inc << Normal(0,1), Normal(0,1), Normal(0,1);
  //rotation_from_euler_angles(end_rot, inc(0), inc(1), inc(2));
  Thread* sample =new Thread(vertices, angles, start_rot, goal_thread->rest_length());
  //sample->set_end_constraint(vertices[vertices.size()-1], end_rot);
	sample->set_end_twist(drand48()*4*M_PI - 2*M_PI);
  sample->unviolate_total_length_constraint();
  sample->project_length_constraint();

  if(hasnan(sample)) {
    return generateSample(goal_thread);
  }

  return sample;
}


Thread* Thread_RRT::getNextGoal() {
  Thread* next_target = NULL; 
  
  if (drand48() < 0.30) {
    //cout << "actual goal" << endl;
//    next->resize(_goal.size());
//    *next = _goal;
//    *next_rot = _goal_rot;
    next_target = new Thread(*(_goal_node->thread)); 
  } else {
    //cout << "random gen" << endl;
    //if (next_thread != _goal_thread) {
    //  delete next_thread;
    // }
    next_target = generateSample(_goal_node->thread);
    //next->project_length_constraint();
    //next->minimize_energy();
//    next_thread = generateSample(_goal_thread);
//    next_thread->minimize_energy();
//    next_thread->toVector(next);
//    *next_rot = next_thread->end_rot();
  }

    //next_thread = next_target;
    return next_target;
}

double Thread_RRT::largeRotation(const Thread* target) {  
  // find the closest node in the tree to next
  VectorXd V_target;
  target->toVector(&V_target);
  RRTNode* closest = findClosestNode(target);

  // choose a random axis, choose a random direction to rotate, rotate by pi/2 in that direction
  Thread* start = new Thread(*(closest->thread)); 

  Vector3d axis;
  double coin = drand48();
  int NUM_STEPS;
  double SMALL_STEP;
  if (coin < 0.33333) {
    cout << "Xaxis " << endl;
    axis = Vector3d::UnitX();
    SMALL_STEP = M_PI/4.0;
    NUM_STEPS = 8;
  } else if (coin < 0.666666) {
    cout << "Yaxis " << endl;
    axis = Vector3d::UnitY();
    SMALL_STEP = M_PI/9.0;
    NUM_STEPS = 3;
  } else {
    cout << "Zaxis " << endl;
    axis = Vector3d::UnitZ();
    SMALL_STEP = M_PI/9.0;
    NUM_STEPS = 3;
  }

  int rot_dir;
  coin = drand48();
  if (coin < 0.5) {
    rot_dir = -1;
  } else {
    rot_dir = 1;
  }


  Vector3d end_pos = closest->endPosition();
  Matrix3d end_rot = closest->endRotation();
  Vector3d translation = (V_target.segment<3>(V_target.size()-3) - end_pos).normalized();

  translation *= Normal(0,2)+2.0/NUM_STEPS;

  // compute rotation about given axis
  Matrix3d rotation(Eigen::AngleAxisd(rot_dir*SMALL_STEP, end_rot*axis));
  vector<Two_Motions*> tmpMotions;
  for(int i = 0; i < NUM_STEPS; i++) {
    //Two_Motions* toMove = new Two_Motions(translation, rotation);
    // apply the motion
    //tmpMotions.push_back(toMove);
    //toMove->applyMotion(end_pos, end_rot);

    start->set_end_constraint(end_pos, end_rot);
    start->minimize_energy();
  }

  cout << " attaching new node: " << endl;
  RRTNode* toadd = new RRTNode(start);
  toadd->prev = closest;
  //start->getTwists(&toadd->twists);
  //toadd->endrot = start->end_rot();
  //toadd->lstMotions = tmpMotions;
  //_tree.push_back(toadd);
  insertIntoRRT(toadd);
  cout << "tree size: " << _tree.size() << endl;
  // return the distance of the new point to goal
  cout << utils.distanceBetween(toadd, (RRTNode*) _goal_node) << endl;
  return utils.distanceBetween(toadd, (RRTNode *) _goal_node);
}

double Thread_RRT::extendToward(Thread* target) {
  
  // find the node closest to the target and extract the end position and rotation

  RRTNode* closest; 
  do {

    closest = findClosestNode(target);
    if (closest == NULL) {
      cout << "Nearest neighbor returned null. Generating new target" << endl;
      if (target == _goal_node->thread) { 
        cout << "Goal thread buggy" << endl; 
        exit(0);
      }
      target = generateSample(_goal_node->thread); 
      //next_thread = target;
    }
  } while (closest == NULL); 
  
  //cout << "closest node acquired" << endl; 
  
  // create a new thread based on the closest
  Thread* start = new Thread(*(closest->thread)); 
  VectorXd motion;
  //interpolation
  //simpleInterpolation(start, target, tmpMotions);

  // solve and apply control to the closest thread
  //cout << "calling SLC" << endl;
  solveLinearizedControl(start, target, motion, START_AND_END);
  //cout << "done SLC" << endl; 
  //solveLinearizedControl(start, target, tmpMotions, END); 
  start->minimize_energy();
  //cout << "done minimize" << endl; 

  RRTNode* toadd = new RRTNode(start); 
  /*if (utils.distanceBetween(toadd, closest) < 5e-1) { 
    closest->CVF += 1;
    totalPenalty += 1;
    //delete toadd; 
    //cout << "[Total penalty on node, Total Penalty]: [ " << closest->CVF  
    //    << ", " << totalPenalty << "]" << endl;
    return DBL_MAX;
  }*/

  //cout << " attaching new node: " << endl;
  toadd->prev = closest;
  toadd->motion = motion;
  insertIntoRRT(toadd);
  
  if (_tree.size() % 100 == 0) { 
    cout << "tree size: " << _tree.size() << endl;
  }


  // find distance of node to goal 
  double scoreToGoal = utils.distanceBetween(toadd, (RRTNode *) _goal_node);

  if (scoreToGoal < bestDist) { 
      bestDist = scoreToGoal; 
      //cout << "Best Dist " << scoreToGoal << endl;
      cout << "Current Best Dist: " <<
        utils.l2PointsDifference(toadd, (RRTNode *) _goal_node) << endl; 
  }

  // return the distance of the new point to target
  RRTNode* targetNode = new RRTNode(target); 
  double scoreToTarget = utils.distanceBetween(toadd, targetNode);
  delete targetNode;

  return scoreToTarget;
}

double Thread_RRT::extendAsFarToward(Thread* target) {
  double prevScore = DBL_MAX;
  double score = DBL_MAX;
  do {
    prevScore = score; 
    score = extendToward(target);
  } while(prevScore - score > 5e-1); 
  return score; 
}

RRTNode* Thread_RRT::findClosestNode(const Thread* target, bool approximateNode) { 
  omp_set_lock(&writelock);

  RRTNode* bestNode = NULL;
  if (approximateNode) { 
    RRTNode* targetNode = new RRTNode(target); 
    bestNode = (RRTNode *) index->query(targetNode);
    delete targetNode;
  } else {
    double bestDist = DBL_MAX;
    for (int i = 0; i < _tree.size(); i++) { 
      if (utils.distanceBetween((RRTNode *) _tree[i], (RRTNode *) _goal_node) 
          < bestDist) {
        bestDist = utils.distanceBetween((RRTNode *) _tree[i], (RRTNode *)_goal_node);
        bestNode = _tree[i]; 
      }
    }
  }
  omp_unset_lock(&writelock);
  return bestNode;
}

void Thread_RRT::insertIntoRRT(RRTNode* node) {
  omp_set_lock(&writelock);
  _tree.push_back(node);
  index->insert(node); 
  omp_unset_lock(&writelock);
}




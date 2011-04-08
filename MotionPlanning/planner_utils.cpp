#include "planner_utils.h"
#include <time.h>
#include <omp.h>


omp_lock_t writelock; 

Thread_RRT::Thread_RRT()
{
}

Thread_RRT::~Thread_RRT()
{
}

void Thread_RRT::initialize(const Thread* start, const Thread* goal)
{

  omp_init_lock(&writelock); 
  _start_node = new RRTNode(start);
  _goal_node = new RRTNode(goal);  

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
  param->repeat = 20;

  index->init(*param, 2); 

  insertIntoRRT((RRTNode *) _start_node); 


}
void Thread_RRT::planStep(Thread& new_sample_thread, Thread& closest_sample_thread, Thread& new_extend_thread) { 
  //cout << "getting target" << endl;
  Thread* next_target = NULL; 
  double newSampleDist = DBL_MAX;
  while (newSampleDist == DBL_MAX) { 
    next_target = getNextGoal();
    //cout << "extending to target" << endl;
    if (drand48() < 1.0) {
      newSampleDist = extendAsFarToward(next_target);
    } else {
      newSampleDist = largeRotation(next_target);
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
  

  cout << increasedDimThread->num_pieces() << endl; 

  return increasedDimThread;

}

Thread* Thread_RRT::halfDimApproximation(const Thread* target) {

  cout << "Start with p: " << target->num_pieces() << endl;

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


  cout << reducedDimensionThread->num_pieces() << endl; 

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

  } while(((goal+noise_goal)-(start+noise_start)).norm() > max_thread_length);

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


  Matrix3d rot = Matrix3d::Identity();
  return new Thread(vertices, angles, rot, goal_thread->rest_length());

}

Thread* Thread_RRT::generateSample(int N) {
  
  vector<Vector3d> vertices;
  vector<double> angles;
  
  vertices.push_back(Vector3d::Zero());
  angles.push_back(0.0);
  int _rest_length = 3;

  vertices.push_back(Vector3d::UnitX()*_rest_length);
  angles.push_back(0.0);

  double angle;
  Vector3d inc;
  
  Vector3d goal; 
  Vector3d noise; 
  double max_thread_length = (N-2)*_rest_length; 

  do {
    goal = (noise <<  Normal(0,1)*max_thread_length,
        Normal(0,1)*max_thread_length,
        Normal(0,1)*max_thread_length).finished();

  } while (goal.norm() > max_thread_length); 


  do {
    inc << Normal(0,1), Normal(0,1), Normal(0,1);
    inc.normalize();
    inc *= _rest_length;
    angle = acos(inc.dot(goal)/(goal.norm()*inc.norm()));
  } while(abs(angle) > M_PI/2.0);


  for(int i = 0; i < N-2; i++) {
    Vector3d prevInc = inc;
    vertices.push_back(vertices[vertices.size()-1] + inc);
    angles.push_back(0.0);
    // time to move toward the goal
    if ((vertices[vertices.size()-1] - goal).squaredNorm() > (N-2-i-1)*(N-2-i-1)*_rest_length*_rest_length) {
      inc = (goal - vertices[vertices.size()-1]).normalized()*_rest_length;
      if ( acos(inc.dot(prevInc) / (prevInc.norm() * inc.norm())) > 3*M_PI/4 ) {
        inc = prevInc; 
      }
    } else {
      if (drand48() < 0.25) {
        if (drand48() < 0.3) {
         do {
            inc << Normal(0,1), Normal(0,1), Normal(0,1);
            inc.normalize();
            inc *= _rest_length;
            angle = acos(inc.dot(goal) / (goal.norm()*inc.norm()));     
          } while(abs(angle) > M_PI/8.0);
        } else {
         do {
            inc << Normal(0,1), Normal(0,1), Normal(0,1);
            inc.normalize();
            inc *= _rest_length;
            angle = acos(inc.dot(prevInc) / (prevInc.norm()*inc.norm()));     
          } while(abs(angle) > M_PI/8.0);
        }
          //cout << angle << endl;
      }
    }
  }


  Matrix3d rot = Matrix3d::Identity();
  Thread* newSample = new Thread(vertices, angles, rot, _rest_length); 
  newSample->minimize_energy();
  return newSample;

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

void Thread_RRT::simpleInterpolation(Thread* start, const Thread* goal, vector<Two_Motions*>& motions) {
  // use quaternion interpolation to move closer to end rot
  // figure out angle between quats, spherically interpolate.
  Vector3d translation;
  Matrix3d rotation; 
  Vector3d cur_pos = start->end_pos();
  Matrix3d cur_rot = start->end_rot();

  Vector3d next_pos = goal->end_pos();
  Matrix3d next_rot = goal->end_rot();

  Matrix3d goal_rot = next_rot;
  Eigen::Quaterniond endq(cur_rot);
  Eigen::Quaterniond goalq(goal_rot);

  Vector3d after_goal = goal_rot*Vector3d::UnitX();
  Vector3d after_end = cur_rot*Vector3d::UnitX();
  double angle = acos(after_goal.dot(after_end));
  double t = M_PI/8.0/angle;
  Eigen::Quaterniond finalq = endq.slerp(t, goalq).normalized();
  rotation = (finalq*endq.inverse()).toRotationMatrix();


  // use linear interpolation to move closer to end pos
  Vector3d goal_pos = next_pos;
  translation = goal_pos - cur_pos;
  double step = 3.0;
  if(translation.squaredNorm() > 0) {
    translation.normalize();
  }
  translation *= step;

  //Two_Motions *toMove = new Two_Motions(translation, rotation);
  //motions.push_back(toMove);
  //toMove->applyMotion(cur_pos, cur_rot);
  start->set_end_constraint(cur_pos, cur_rot); 

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
      //delete target;
      cout << target << endl;
      target = generateSample(_goal_node->thread); 
      //next_thread = target;
    }
  } while (closest == NULL); 
  
  
  // create a new thread based on the closest
  Thread* start = new Thread(*(closest->thread));
  start->set_rest_length(closest->thread->rest_length());


  Vector3d translation;
  Matrix3d rotation;
  vector<Two_Motions*> tmpMotions;

  //interpolation
  //simpleInterpolation(start, target, tmpMotions);

  // solve and apply control to the closest thread
  solveLinearizedControl(start, target, tmpMotions, START_AND_END);
  //solveLinearizedControl(start, target, tmpMotions, END); 
  start->minimize_energy();


  RRTNode* toadd = new RRTNode(start); 
  if (utils.distanceBetween(toadd, closest) < 5e-1) { 
    closest->CVF += 1;
    totalPenalty += 1;
    //delete toadd; 
    //cout << "[Total penalty on node, Total Penalty]: [ " << closest->CVF  
    //    << ", " << totalPenalty << "]" << endl;
    return DBL_MAX;
  }

  //cout << " attaching new node: " << endl;
  toadd->prev = closest;
  toadd->lstMotions = tmpMotions;
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
  //delete targetNode;

  return scoreToTarget;
}

double Thread_RRT::extendAsFarToward(Thread* target) {
  double prevScore = DBL_MAX;
  double score = DBL_MAX;
  do {
    prevScore = score; 
    score = extendToward(target);
  } while(prevScore - score > 1e-1); 
  return score; 
}


/*double Thread_RRT::distanceBetween(const Thread* start, const Thread* end) {
  int N = start->num_pieces();
  Thread *st = (Thread *) start;
  Thread *en = (Thread *) end; 

  
  double cost = 0.0; 
  VectorXd startV, endV; 
  start->toVector(&startV);
  end->toVector(&endV);
  cost += (startV - endV).norm();

  VectorXd startGradientV, endGradientV;
  startGradientV.resize(3*N);
  endGradientV.resize(3*N);
  st->calculate_gradient_vertices_vectorized(&startGradientV);
  en->calculate_gradient_vertices_vectorized(&endGradientV);
  //cost += (startGradientV - endGradientV).norm();

  return cost; 
  //return (startV - endV).cwise().abs().sum();
} */


RRTNode* Thread_RRT::findClosestNode(const Thread* target, bool approximateNode) { 
  omp_set_lock(&writelock);

  RRTNode* bestNode = NULL;
  if (approximateNode) { 
    RRTNode* targetNode = new RRTNode(target); 
    bestNode = (RRTNode *) index->query(targetNode); 
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



/*RRTNode* Thread_RRT::findClosestNode(const Thread* target) {
  double bestDist = DBL_MAX;
  double norm = 0.0;
  RRTNode* ptr = NULL;
  for(int i = 0; i < _tree.size(); i++) {
    //if (_tree[i]->CVF / totalPenalty < drand48()) {  
      norm = distanceBetween(&(_tree[i]->thread), target); 
      // norm = (next - _tree[i]->x).squaredNorm();
      // norm = (next - _tree[i]->x).cwise().abs().sum();
      if (norm < bestDist) {
        ptr = _tree[i];
        be
stDist = norm;
      }
    //}
  }
  if (target == _goal_thread) { 
    cout << "best dist: " << bestDist << endl;
  }
  return ptr;
}*/ 

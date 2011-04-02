#include "planner_utils.h"
#include <time.h>

Thread_RRT::Thread_RRT()
{
}

Thread_RRT::~Thread_RRT()
{
}

void Thread_RRT::initialize(const Thread* start, const Thread* goal)
{

  _start_thread = start;
  _goal_thread = goal;
  goal->toVector(&_goal);
  _goal_rot = goal->end_rot();

  for (int i=0; i < _tree.size(); i++)
  {
    delete _tree[i];
  }
  _tree.clear();

  RRTNode* startNode = new RRTNode(start);
  _tree.push_back(startNode);

//  VectorXd test;
//  start->toVector(&test);

  distToGoal = DBL_MAX;
  bestDist = DBL_MAX;
  TOLERANCE = 0.01;
  totalPenalty = 1.0; 
  //next.resize(_goal.size());
  //next_rot.setZero();
  next_thread = NULL; 

  index = new LshTable<HASH>();
  HASH::Parameter *param = new HASH::Parameter();
  unsigned int dim = start->num_pieces()*3;
  param->dim = dim;
  param->repeat = 20;

  index->init(*param, 4); 
  index->insert(startNode); 


}
void Thread_RRT::planStep(Thread& new_sample_thread, Thread& closest_sample_thread, Thread& new_extend_thread) { 
//void Thread_RRT::planStep(VectorXd* vertices, VectorXd* prev, VectorXd* next_thread) {
  //cout << "getting target" << endl;
  double newSampleDist = DBL_MAX;
  while (newSampleDist == DBL_MAX) { 
    getNextGoal(next_thread);
    //cout << "extending to target" << endl;
    if (drand48() < 1.0) {
      newSampleDist = extendAsFarToward(next_thread);
    } else {
      newSampleDist = largeRotation(next_thread);
    }
  }

  new_sample_thread = *next_thread;
  closest_sample_thread = (_tree.back()->prev->thread);
  new_extend_thread = (_tree.back()->thread);
  //*prev = (_tree.back()->prev->x);
  //*next_thread = (_tree.back()->x);
  //cout << "done step" << endl;

  RRTNode* closest = findClosestNode(_goal_thread);
  while(closest->prev != NULL) {
    closest->prev->next = closest;
    closest = closest->prev;
  }
}

/*void Thread_RRT::planPath(const Thread* start, const Thread* goal, vector<Frame_Motion>& movements) {
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
  
  vertices.push_back(Vector3d::Zero());
  angles.push_back(0.0);

  vertices.push_back(Vector3d::UnitX()*_rest_length);
  angles.push_back(0.0);

  double angle;
  Vector3d inc;
  
  Vector3d goal; 
  Vector3d noise; 
  double max_thread_length = (N-2)*_rest_length; 
  if (drand48() < 0.3) { 
  
  // sample the goal by taking the current goal and adding noise
  // these samples are very similar to the goal 
    //cout << "sampling near the goal" << endl; 
    goal = goal_thread->end_pos() - goal_thread->start_pos();
    do { 
    noise = (noise << Normal(0,1)*_rest_length,
      Normal(0,1)*_rest_length,
      Normal(0,1)*_rest_length).finished();
    } while((goal+noise).norm() > max_thread_length);

    goal += noise; 

  } else { 
    // sample the goal by sampling in the sphere of the norm
    // these samples tend to increase the branching of the tree
    //cout << "sampling in the ball" << endl;
    do {
      goal = (noise <<  Normal(0,1)*max_thread_length,
          Normal(0,1)*max_thread_length,
          Normal(0,1)*max_thread_length).finished();

    } while (goal.norm() > max_thread_length); 
  }

  do {
    inc << Normal(0,1), Normal(0,1), Normal(0,1);
    //inc = goal;
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
            //inc = goal;
            inc.normalize();
            inc *= _rest_length;
            angle = acos(inc.dot(goal) / (goal.norm()*inc.norm()));     
          } while(abs(angle) > M_PI/8.0);
        } else {
         do {
            inc << Normal(0,1), Normal(0,1), Normal(0,1);
            //inc = goal;
            inc.normalize();
            inc *= _rest_length;
            angle = acos(inc.dot(prevInc) / (prevInc.norm()*inc.norm()));     
          } while(abs(angle) > M_PI/8.0);
        }
          //cout << angle << endl;
      }
    }
  }

  for (int i = 0; i < N; i++) {
    for (int j = 0; j < 3; j++) { 
      cout << vertices[i][j] << " ";
    }
    cout << endl; 
  }

  Matrix3d rot = Matrix3d::Identity();
  return new Thread(vertices, angles, rot);


}



//void Thread_RRT::getNextGoal(VectorXd* next, Matrix3d* next_rot) {
void Thread_RRT::getNextGoal(Thread* next) {
  if (drand48() < 0.10) {
    //cout << "actual goal" << endl;
//    next->resize(_goal.size());
//    *next = _goal;
//    *next_rot = _goal_rot;
    next = (Thread *) _goal_thread; 
  } else {
    //cout << "random gen" << endl;
    //if (next_thread != _goal_thread) {
    //  delete next_thread;
    //}
    next = generateSample(_goal_thread);
    next->minimize_energy();
//    next_thread = generateSample(_goal_thread);
//    next_thread->minimize_energy();
//    next_thread->toVector(next);
//    *next_rot = next_thread->end_rot();
  }

    next_thread = next;
}

void Thread_RRT::simpleInterpolation(const Thread* start, const Thread* goal, Vector3d* translation, Matrix3d* rotation) {
//void Thread_RRT::simpleInterpolation(const Vector3d& cur_pos, const Matrix3d& cur_rot, const Vector3d& next_pos, const Matrix3d& next_rot, Vector3d* translation, Matrix3d* rotation) {
  // use quaternion interpolation to move closer to end rot
  // figure out angle between quats, spherically interpolate.
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
  (*rotation) = (finalq*endq.inverse()).toRotationMatrix();


  // use linear interpolation to move closer to end pos
  Vector3d goal_pos = next_pos;
  *translation = goal_pos - cur_pos;
  double step = 3.0;
  if(translation->squaredNorm() > 0) {
    translation->normalize();
  }
  *translation *= step;
}
double Thread_RRT::largeRotation(const Thread* target) {  
//double Thread_RRT::largeRotation(const VectorXd& next) {
  // find the closest node in the tree to next
  VectorXd V_target;
  target->toVector(&V_target);
  RRTNode* closest = findClosestNode(target);

  // choose a random axis, choose a random direction to rotate, rotate by pi/2 in that direction
  //Thread* start = new Thread(closest->x, closest->twists, Matrix3d::Identity());
  Thread start = closest->thread;

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
  vector<Frame_Motion*> tmpMotions;
  for(int i = 0; i < NUM_STEPS; i++) {
    Frame_Motion* toMove = new Frame_Motion(translation, rotation);
    // apply the motion
    tmpMotions.push_back(toMove);
    toMove->applyMotion(end_pos, end_rot);
    start.set_end_constraint(end_pos, end_rot);
    start.minimize_energy();
  }

  cout << " attaching new node: " << endl;
  RRTNode* toadd = new RRTNode(&start);
  toadd->prev = closest;
  //start->getTwists(&toadd->twists);
  //toadd->endrot = start->end_rot();
  toadd->lstMotions = tmpMotions;
  //_tree.push_back(toadd);
  insertIntoRRT(toadd);
  cout << "tree size: " << _tree.size() << endl;
  // return the distance of the new point to goal
  //delete start;
  //return (toadd->x - _goal).squaredNorm();
  cout << distanceBetween(&start, _goal_thread) << endl;
  return distanceBetween(&start, target);
}

double Thread_RRT::extendToward(const Thread* target) {
//double Thread_RRT::extendToward(const VectorXd& next, const Matrix3d& next_rot) {
  //Thread* start = new Thread(closest->x, closest->twists, Matrix3d::Identity());

  // get the end position and end_rotation of target
  Vector3d target_pos = target->end_pos();
  Matrix3d target_rot = target->end_rot();

  // find the node closest to the target and extract the end position and rotation
  RRTNode* closest = findClosestNode(target);
  Vector3d end_pos = closest->endPosition();
  Matrix3d end_rot = closest->endRotation(); 

  // create a new thread based on the closest
  //Thread start = closest->thread;

  if (closest == NULL) {
    cout << "HUH??????" << endl;
  }

  Thread* start = new Thread(closest->thread);
  
  //Vector3d next_pos = next.segment<3>(next.size()-3);
  Vector3d translation;
  Matrix3d rotation;
  vector<Frame_Motion*> tmpMotions;
  //cout << " before motion: " << endl << end_pos << endl << end_rot << endl;
  // apply the motion
  /*simpleInterpolation(&start, target, &translation, &rotation);
    Frame_Motion* toMove = new Frame_Motion(translation, rotation);
    tmpMotions.push_back(toMove);
    toMove->applyMotion(end_pos, end_rot); 

  //cout << " after motion: " << endl << end_pos << endl << end_rot << endl;
  start.set_end_constraint(end_pos, end_rot);
  */ 

  // solve and apply control to the closest thread
  solveLinearizedControl(start, target, tmpMotions); 
  start->minimize_energy();

  if (distanceBetween(start, &(closest->thread)) < 5e-1) { 
    closest->CVF += 1;
    totalPenalty += 1;
    //cout << "[Total penalty on node, Total Penalty]: [ " << closest->CVF  
    //    << ", " << totalPenalty << "]" << endl;
    return DBL_MAX;
  }

  //cout << " attaching new node: " << endl;
  RRTNode* toadd = new RRTNode(start);
  toadd->prev = closest;
  //start->getTwists(&toadd->twists);
  //toadd->endrot = start->end_rot();
  toadd->lstMotions = tmpMotions;
  //_tree.push_back(toadd);
  insertIntoRRT(toadd);
  if (_tree.size() % 100 == 0) { 
    cout << "tree size: " << _tree.size() << endl;
  }
  // return the distance of the new point to goal
  // delete start;
  //return (toadd->x - _goal).squaredNorm();
  return distanceBetween(start, target); 
}

double Thread_RRT::extendAsFarToward(const Thread* target) {
  double prevScore = DBL_MAX;
  double score = DBL_MAX;
  do {
    prevScore = score; 
    score = extendToward(target); 
  } while(prevScore - score > 1e-1); 
  return score; 
}


double Thread_RRT::distanceBetween(const Thread* start, const Thread* end) {
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

RRTNode* Thread_RRT::findClosestNode(const Thread* target) { 
  RRTNode* targetNode = new RRTNode(target); 
  return (RRTNode*) index->query(targetNode); 

}

void Thread_RRT::insertIntoRRT(RRTNode* node) { 
  _tree.push_back(node);
  index->insert(node); 
}




#include "thread_discrete_RRT.h"
#include <time.h>

RRTNode::~RRTNode() {
  for(int i = 0; i < lstMotions.size(); i++) {
    delete lstMotions[i];
  }
  lstMotions.clear();
}

RRTNode::RRTNode(): prev(NULL), next(NULL), linearized(false) {
}

RRTNode::RRTNode(const Thread* start): prev(NULL), next(NULL), linearized(false) {
  start->toVector(&x);
  start->getTwists(&twists);
  B.resize(start->num_pieces()*3, 6);
  endrot = start->end_rot();
  N = x.size();
}


Thread_RRT::Thread_RRT()
{
}

Thread_RRT::~Thread_RRT()
{
}

void Thread_RRT::initialize(const Thread* start, const Thread* goal)
{
  goal->toVector(&_goal);
  _goal_rot = goal->end_rot();

  for (int i=0; i < _tree.size(); i++)
  {
    delete _tree[i];
  }
  _tree.clear();
  _tree.push_back(new RRTNode(start));

  VectorXd test;
  start->toVector(&test);

  distToGoal = DBL_MAX;
  bestDist = DBL_MAX;
  TOLERANCE = 0.01;
  next.resize(_goal.size());
  next_rot.setZero();
}

void Thread_RRT::planStep(VectorXd* vertices, VectorXd* prev, VectorXd* next_thread) {
  cout << "getting target" << endl;
  getNextGoal(&next, &next_rot);
  *vertices = next;
  cout << "extending to target" << endl;
  if (drand48() < 0.7) {
    distToGoal = extendToward(next, next_rot);
  } else {
    distToGoal = largeRotation(next);
  }
  *prev = (_tree.back()->prev->x);
  *next_thread = (_tree.back()->x);
  cout << "done step" << endl;

  RRTNode* closest = findClosestNode(_goal);
  while(closest->prev != NULL) {
    closest->prev->next = closest;
    closest = closest->prev;
  }
}

void Thread_RRT::planPath(const Thread* start, const Thread* goal, vector<Frame_Motion>& movements) {
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
}


void Thread_RRT::getNextGoal(VectorXd* next, Matrix3d* next_rot) {
  if (drand48() < 0.3) {
    cout << "actual goal" << endl;
    next->resize(_goal.size());
    *next = _goal;
    *next_rot = _goal_rot;
  } else {
    cout << "random gen" << endl;
    // generate random
    // randomly stick out in a direction
    int N = _goal.size();
    next->resize(N);

    next->segment<3>(0) = Vector3d::Zero();
    next->segment<3>(3) = Vector3d::UnitX()*_rest_length;


    double angle;
    Vector3d inc;
    Vector3d goal = _goal.segment<3>(N-3) - _goal.segment<3>(0);
    Vector3d noise;
    goal += (noise << Normal(0,1),Normal(0,1),Normal(0,1)).finished()*20.0;

    do {
      inc << Normal(0,1), Normal(0,1), Normal(0,1);
      angle = acos(inc.dot(goal)/(goal.norm()*inc.norm()));
    } while(abs(angle) > M_PI/2.0);

    inc.normalize();
    inc *= _rest_length;
    for(int i = 0; i < N/3-2; i++) {
      next->segment<3>(3*(i+2)) = next->segment<3>(3*(i+1)) + inc;
      if ((next->segment<3>(3*(i+2)) - goal).squaredNorm() > (N/3-2-i-1)*(N/3-2-i-1)*_rest_length*_rest_length) {
        inc = (goal - next->segment<3>(3*(i+2))).normalized()*_rest_length;
      }
    }

    VectorXd twists = VectorXd::Zero(N/3);
    Thread* start = new Thread(*next, twists, Matrix3d::Identity());
    start->minimize_energy();
    start->toVector(next);
    *next_rot = start->end_rot();
    delete start;
  }
}

void Thread_RRT::applyControl(Thread* start, const VectorXd& u, VectorXd* res, Frame_Motion* motion) {
  Vector3d translation;
  translation << u(0), u(1), u(2);

  double dw = 1.0 - u(3)*u(3) - u(4)*u(4) - u(5)*u(5);
  if (dw < 0) {
    cout << "Huge differential quaternion: " << endl;
    cout << u.segment<3>(3) << endl;
  }
  dw = (dw > 0) ? dw : 0.0;
  dw = sqrt(dw);
  Eigen::Quaterniond q(dw, u(3),u(4),u(5));
  Matrix3d rotation(q);

  // apply the control u to thread start, and return the new config in res
  Frame_Motion toMove(translation, rotation);
  *motion = toMove;

  Vector3d end_pos = start->end_pos();
  Matrix3d end_rot = start->end_rot();
  toMove.applyMotion(end_pos, end_rot);
  start->set_end_constraint(end_pos, end_rot);
  start->minimize_energy();

  start->toVector(res);
}

void Thread_RRT::simpleInterpolation(const Vector3d& cur_pos, const Matrix3d& cur_rot, const Vector3d& next_pos, const Matrix3d& next_rot, Vector3d* translation, Matrix3d* rotation) {
  // use quaternion interpolation to move closer to end rot
  // figure out angle between quats, spherically interpolate.
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

double Thread_RRT::largeRotation(const VectorXd& next) {
  // find the closest node in the tree to next
  RRTNode* closest = findClosestNode(next);

  // choose a random axis, choose a random direction to rotate, rotate by pi/2 in that direction
  Thread* start = new Thread(closest->x, closest->twists, Matrix3d::Identity());

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
  Vector3d translation = (next.segment<3>(next.size()-3) - end_pos).normalized();
  translation *= Normal(0,2)+2.0/NUM_STEPS;

  // compute rotation about given axis
  Matrix3d rotation(Eigen::AngleAxisd(rot_dir*SMALL_STEP, end_rot*axis));
  vector<Frame_Motion*> tmpMotions;
  for(int i = 0; i < NUM_STEPS; i++) {
    Frame_Motion* toMove = new Frame_Motion(translation, rotation);
    // apply the motion
    tmpMotions.push_back(toMove);
    toMove->applyMotion(end_pos, end_rot);
    start->set_end_constraint(end_pos, end_rot);
    start->minimize_energy();
  }

  cout << " attaching new node: " << endl;
  RRTNode* toadd = new RRTNode(start);
  toadd->prev = closest;
  start->getTwists(&toadd->twists);
  toadd->endrot = start->end_rot();
  toadd->lstMotions = tmpMotions;
  _tree.push_back(toadd);
  cout << "tree size: " << _tree.size() << endl;
  // return the distance of the new point to goal
  delete start;
  return (toadd->x - _goal).squaredNorm();
}

double Thread_RRT::extendToward(const VectorXd& next, const Matrix3d& next_rot) {
  // find the closest node in the tree to next
  RRTNode* closest = findClosestNode(next);

  // extend closest node toward next
  Thread* start = new Thread(closest->x, closest->twists, Matrix3d::Identity());

  Vector3d end_pos = closest->endPosition();
  Matrix3d end_rot = closest->endRotation();
  Vector3d next_pos = next.segment<3>(next.size()-3);
  Vector3d translation;
  Matrix3d rotation;
  vector<Frame_Motion*> tmpMotions;
  simpleInterpolation(end_pos, end_rot, next_pos, next_rot, &translation, &rotation);
  //cout << " before motion: " << endl << end_pos << endl << end_rot << endl;
  // apply the motion
  Frame_Motion* toMove = new Frame_Motion(translation, rotation);
  tmpMotions.push_back(toMove);
  toMove->applyMotion(end_pos, end_rot);
  //cout << " after motion: " << endl << end_pos << endl << end_rot << endl;
  start->set_end_constraint(end_pos, end_rot);
  start->minimize_energy();


  cout << " attaching new node: " << endl;
  RRTNode* toadd = new RRTNode(start);
  toadd->prev = closest;
  start->getTwists(&toadd->twists);
  toadd->endrot = start->end_rot();
  toadd->lstMotions = tmpMotions;
  _tree.push_back(toadd);
  cout << "tree size: " << _tree.size() << endl;
  // return the distance of the new point to goal
  delete start;
  return (toadd->x - _goal).squaredNorm();

  // add to tree
  // return distance to goal

  // extend that closest node toward next
  // Thread* start = new Thread(closest->x, closest->twists, Matrix3d::Identity());

  // if (!closest->linearized) {
  //   cout << "linearizing" << endl;
  //   // compute linearization
  //   closest->B.setZero();
  //   VectorXd du(6);
  //   double eps = 0.01;

  //   start->save_thread_pieces();
  //   VectorXd plus(next.size());
  //   VectorXd minus(next.size());
  //   Frame_Motion dummy;
  //   for(int i = 0; i < 6; i++) {
  //     du.setZero();
  //     du(i) = eps;
  //     start->restore_thread_pieces();
  //     applyControl(start, du, &plus, &dummy);

  //     du(i) = -eps;
  //     start->restore_thread_pieces();
  //     applyControl(start, du, &minus, &dummy);

  //     closest->B.col(i) = (plus - minus) / (2*eps);
  //   }
  //   start->restore_thread_pieces();
  //   closest->linearized = true;
  //   cout << "linearized" << endl;
  // }

  // // calculate frame motions, set prev ptr
  // VectorXd dx;
  // dx = next - closest->x;
  // double C = 2.0;
  // VectorXd u = closest->B.transpose()*dx;
  // (closest->B.transpose()*closest->B).llt().solveInPlace(u);
  // u *= C/(closest->B*u).squaredNorm();


  // VectorXd x;
  // Frame_Motion* motion = new Frame_Motion;
  // applyControl(start, u, &x, motion);

  // RRTNode* toadd = new RRTNode(start);
  // toadd->prev = closest;
  // toadd->motion = motion;
  // _tree.push_back(toadd);
  // // return the distance of the new point to goal
  // delete start;
  // return (toadd->x - _goal).squaredNorm();
}

RRTNode* Thread_RRT::findClosestNode(const VectorXd& next) {
  double bestDist = DBL_MAX;
  double norm = 0.0;
  RRTNode* ptr = NULL;
  for(int i = 0; i < _tree.size(); i++) {
    norm = (next - _tree[i]->x).squaredNorm();
    // norm = (next - _tree[i]->x).cwise().abs().sum();
    if (norm < bestDist) {
      ptr = _tree[i];
      bestDist = norm;
    }
  }
  cout << "best dist: " << bestDist << endl;
  return ptr;
}

// void Thread_RRT::planPath(const Thread* start, const Thread* goal, vector<Thread_Motion>& movements, vector<Thread*>& intermediateThread)
// {
//   while (!goalReached && currNodes.size() < MAX_NUM_NODES)
//   {
//     //find the nearest neighbor
//     int indClosest;
//     findClosestNode(currGoal, indClosest);

//     //find motion
//     Thread_Motion* nextMotion = new Thread_Motion();
//     findThreadMotion(currNodes[indClosest]->this_node, currGoal, *nextMotion);

//     //add new node
//     currNodes.push_back(new RRT_Node_And_Edge(currNodes[indClosest], nextMotion));
//     currNodes[indClosest]->this_node.applyMotion(currNodes.back()->this_node, *nextMotion);

//     double thisDistToGoal = goalNode.distanceToNode(currNodes.back()->this_node);
//     if (thisDistToGoal < bestDistToGoal)
//     {
//       bestDistToGoal = thisDistToGoal;
//       std::cout << "new best dist: " << bestDistToGoal << std::endl;
//     }

//     if (goalNode.distanceToNode(currNodes.back()->this_node) < GOAL_DISTANCE_THRESH)
//     {
//       goalReached = true;
//     }

//   }






// void Thread_RRT::getNextGoal(RRT_Node& goal, RRT_Node& goalThisIter)
// {
//   //first, see if we are sampling randomly, or around goal
//   double randNum = ((double)rand()) / ((double)RAND_MAX);
//   if (randNum < RANDOM_SAMPLE_THRESH)
//   {
//     setThisGoalNearGoal(goal, goalThisIter);
//     return;
//   }

//   //sample random goal
//   Matrix4d startTrans;
//   goal.thread->getStartTransform(startTrans);

//   Vector3d startPos = startTrans.corner(Eigen::TopRight,3,1);
//   Vector3d startTan = startTrans.corner(Eigen::TopLeft,3,1);
//   goalThisIter.thread = new Thread(goal.thread->length(), startPos, startTan);

//   goalThisIter.thread->minimize_energy();
//   goalThisIter.setPoints();
// }

// void Thread_RRT::setThisGoalNearGoal(RRT_Node& goal, RRT_Node& goalThisIter)
// {
//   //sample something in sphere between current point and goal
//   //first, set the things we know
//   Matrix4d startTrans;
//   goal.thread->getStartTransform(startTrans);
//   Vector3d posNewThread[2];
//   Vector3d tanNewThread[2];
//   double lengthNewThread[2];
//   double length_thread = goal.thread->length();
//   lengthNewThread[0] = lengthNewThread[1] = length_thread/2.0;
//   posNewThread[0] = startTrans.corner(Eigen::TopRight,3,1);
//   tanNewThread[0] = startTrans.corner(Eigen::TopLeft,3,1);
//   Vector3d endPos;
//   goal.thread->getWantedEndPosition(endPos);
//   Vector3d endTan;
//   goal.thread->getWantedEndTangent(endTan);


//   //random params for initializing
//   double curvatureNewThread[2] = {randomNumUnit()*RANDOM_THREAD_MAX_CURVATURE_INIT, randomNumUnit()*RANDOM_THREAD_MAX_CURVATURE_INIT};
//   double torsionNewThread[2] = {randomNumUnit()*RANDOM_THREAD_MAX_TORSION_INIT, randomNumUnit()*RANDOM_THREAD_MAX_TORSION_INIT};

//   //find closest thread to goal
//   int indClosest;
//   findClosestNode(goal, indClosest);

//   Vector3d endPosClosestToGoal;
//   currNodes[indClosest]->this_node.thread->getWantedEndPosition(endPosClosestToGoal);
//   Vector3d endTanClosestToGoal;
//   currNodes[indClosest]->this_node.thread->getWantedEndTangent(endTanClosestToGoal);

//   //calculate the distance to this end
//   //sample from this radius, add to end point
//   double randX, randY, randZ, sampledRadius;
//   double radius_pos = (endPos - endPosClosestToGoal).norm() + ADD_TO_GOAL_DIST_RADIUS;
//   double dist_new_thread;
//   do
//   {
//     //find point in sphere with this radius, add to goal endpoint
//     double sampledRadius;
//     do
//     {
//       randX = randomMaxAbsValue(radius_pos);
//       randY = randomMaxAbsValue(radius_pos);
//       randZ = randomMaxAbsValue(radius_pos);
//       sampledRadius = randX*randX + randY*randY + randZ*randZ;
//     } while (sampledRadius >= (radius_pos*radius_pos));
//     posNewThread[1](0) = randX+endPos(0);
//     posNewThread[1](1) = randY+endPos(1);
//     posNewThread[1](2) = randZ+endPos(2);

//     dist_new_thread = (posNewThread[1]-posNewThread[0]).norm();
//   } while (dist_new_thread >= length_thread);

//   //sample from a tan around the goal tan
//   //sample from outside of sphere with radius equal to length between goal tan and closest tan
//   //normalize the length of that new tan
//   double radius_tan = (endTan - endTanClosestToGoal).norm() + ADD_TO_GOAL_TAN_RADIUS;
//   double randTheta = M_PI*randomNumUnit();
//   double randPhi = 2.0*M_PI*randomNumUnit();
//   tanNewThread[1](0) = endTan(0)+radius_tan*cos(randTheta)*sin(randPhi);
//   tanNewThread[1](1) = endTan(1)+radius_tan*sin(randTheta)*sin(randPhi);
//   tanNewThread[1](2) = endTan(2)+radius_tan*cos(randPhi);

//   tanNewThread[1].normalize();

// //  std::cout << "goal end pos: " << endPos << std::endl;
// //  std::cout << "goal end tan: " << endTan << std::endl;

// //  std::cout << "new end pos: " << posNewThread[1] << std::endl;
// //  std::cout << "new end tan: " << tanNewThread[1] << std::endl;


//   goalThisIter.thread = new Thread(goal.thread);
//   goalThisIter.thread->setEndConstraint(posNewThread[1], tanNewThread[1]);
//   goalThisIter.thread->upsampleAndOptimize_minLength(0.065);
//   goalThisIter.thread->minimize_energy();


//   //goalThisIter.thread = new Thread(curvatureNewThread, torsionNewThread, lengthNewThread, 2, posNewThread, tanNewThread);
//   //goalThisIter.thread = new Thread(length_thread, posNewThread[0], tanNewThread[0]);
//   //goalThisIter.thread->minimize_energy();
//   goalThisIter.setPoints();

//   double distToGoal = goal.distanceToNode(goalThisIter);
//   //std::cout << "distance to actual goal: " << distToGoal << std::endl;

// }



// double Thread_RRT::findClosestNode(RRT_Node& goalNode, int& ind_to_closest)
// {
//   ind_to_closest = 0;
//   double dist = goalNode.distanceToNode(currNodes[0]->this_node);
//   for (int i=1; i < currNodes.size(); i++)
//   {
//     double thisDist = goalNode.distanceToNode(currNodes[i]->this_node);
//     if (thisDist < dist)
//     {
//       dist = thisDist;
//       ind_to_closest = i;
//     }
//   }

//   return dist;
// }

// void Thread_RRT::findThreadMotion(RRT_Node& start, RRT_Node& goal, Thread_Motion& bestMotion)
// {
//   //find vector to get position closer
//   Vector3d startEnd;
//   start.thread->getWantedEndPosition(startEnd);
//   Vector3d goalEnd;
//   goal.thread->getWantedEndPosition(goalEnd);

//   bestMotion.pos_movement = goalEnd - startEnd;

//   //add some noise to movement
//   bestMotion.pos_movement(0) += randomMaxAbsValue(MAX_NOISE_DISTANCE_MOVEMENT);
//   bestMotion.pos_movement(1) += randomMaxAbsValue(MAX_NOISE_DISTANCE_MOVEMENT);
//   bestMotion.pos_movement(2) += randomMaxAbsValue(MAX_NOISE_DISTANCE_MOVEMENT);

//   //make sure vector isn't too long (that's what she said)
//   double normOfVec = bestMotion.pos_movement.norm();
//   if (normOfVec > MAX_DISTANCE_MOVEMENT)
//   {
//     bestMotion.pos_movement *= (MAX_DISTANCE_MOVEMENT/normOfVec);
//   }

//   //rotate towards other tan
//   Vector3d startTan;
//   start.thread->getWantedEndTangent(startTan);
//   Vector3d goalTan;
//   goal.thread->getWantedEndTangent(goalTan);

//   Vector3d vecToRotateAbout = startTan.cross(goalTan);
//   vecToRotateAbout.normalize();
//   //add noise to tan
//   vecToRotateAbout(0) += randomMaxAbsValue(MAX_NOISE_ROTATION_VECTOR);
//   vecToRotateAbout(1) += randomMaxAbsValue(MAX_NOISE_ROTATION_VECTOR);
//   vecToRotateAbout(2) += randomMaxAbsValue(MAX_NOISE_ROTATION_VECTOR);
//   vecToRotateAbout.normalize();

//   double angToRotate = min(MAX_ANG_TO_ROTATE, acos(startTan.dot(goalTan)));
//   angToRotate += randomMaxAbsValue(MAX_NOISE_ROTATION_ANGLE);

//   bestMotion.tan_rotation = Eigen::AngleAxisd( angToRotate, vecToRotateAbout);

// }





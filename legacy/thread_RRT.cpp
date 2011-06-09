#include "thread_RRT.h"
#include <time.h>


Thread_RRT::Thread_RRT()
{
  currNodes.reserve(MAX_NUM_NODES);


}



Thread_RRT::~Thread_RRT()
{

}


void Thread_RRT::planPath(const Thread* start, const Thread* goal, vector<Thread_Motion>& movements, vector<Thread*>& intermediateThread)
{
  RRT_Node startNode = RRT_Node(start);
  RRT_Node goalNode = RRT_Node(goal);

  initializeSearch(startNode, goalNode);
  intermediateThread.push_back(new Thread(start));
  intermediateThread.push_back(new Thread(goal));

  /*
  intermediateThread.push_back(new Thread(start));
  MatrixXd resampled_1(33,3);
  startNode.thread->getPoints(resampled_1);
  MatrixXd resampled_2(17,3);
  startNode.thread->getPoints(resampled_2);
  intermediateThread.push_back(new Thread(startNode.thread, 17, 2));
  intermediateThread.push_back(new Thread(resampled_2, 2, startNode.thread->length()));*/

  bool goalReached = false;
  double bestDistToGoal = DBL_MAX;
  while (!goalReached && currNodes.size() < MAX_NUM_NODES)
  {
    std::cout << "curr ind" << currNodes.size() << std::endl;
    //get a goal for this iteration
    RRT_Node currGoal;
    getNextGoal(goalNode, currGoal);

    //find the nearest neighbor
    int indClosest;
    findClosestNode(currGoal, indClosest);

    //find motion
    Thread_Motion* nextMotion = new Thread_Motion();
    findThreadMotion(currNodes[indClosest]->this_node, currGoal, *nextMotion);

    //add new node
    currNodes.push_back(new RRT_Node_And_Edge(currNodes[indClosest], nextMotion));
    currNodes[indClosest]->this_node.applyMotion(currNodes.back()->this_node, *nextMotion);

    double thisDistToGoal = goalNode.distanceToNode(currNodes.back()->this_node);
    if (thisDistToGoal < bestDistToGoal)
    {
      bestDistToGoal = thisDistToGoal;
      std::cout << "new best dist: " << bestDistToGoal << std::endl;
    }

    if (goalNode.distanceToNode(currNodes.back()->this_node) < GOAL_DISTANCE_THRESH)
    {
      goalReached = true; 
    }



  }

  int finalIndClosest;
  findClosestNode(goalNode, finalIndClosest);

  //count how many movements we need for the best solution
  int numThreadsBetween = -1;
  RRT_Node_And_Edge* currPtr = currNodes[finalIndClosest];
  while (currPtr != NULL)
  {
    numThreadsBetween ++;
    currPtr = currPtr->last_node;
  }


  //set the movements and intermediate threads for return
  movements.resize(numThreadsBetween);
  intermediateThread.resize(numThreadsBetween);

  int movementInd = numThreadsBetween-1;
  currPtr = currNodes[finalIndClosest];
  while (movementInd >= 0)
  {
    movements[movementInd] = *(currPtr->motion_from_last);
    intermediateThread[movementInd] = currPtr->this_node.thread;
    currPtr->this_node.thread = NULL;     //so we don't delete it!
    
    movementInd--;
    currPtr = currPtr->last_node;
  } 



  /*intermediateThread.resize(currNodes.size());
  for (int i=0; i < intermediateThread.size(); i++)
  {
    intermediateThread[i] = new Thread(currNodes[i]->this_node.thread);
  }*/

  //cleanup();

 /* 
  movements.resize(5);
  movements[0].pos_movement = Vector3d(7.0, -5.0, 1.0);
  movements[0].tan_rotation = Eigen::AngleAxisd (M_PI/90.0, Vector3d(1.0, 0.0, 0.0));
  movements[1].pos_movement = Vector3d(7.0, -4.0, 2.0);
  movements[1].tan_rotation = Eigen::AngleAxisd (M_PI/100.0, Vector3d(1.0, 0.2, 0.0));
  movements[2].pos_movement = Vector3d(6.0, -6.0, 3.0);
  movements[2].tan_rotation = Eigen::AngleAxisd (M_PI/100.0, Vector3d(1.0, 0.0, 0.2));
  movements[3].pos_movement = Vector3d(8.0, -4.0, 3.0);
  movements[3].tan_rotation = Eigen::AngleAxisd (M_PI/80.0, Vector3d(1.0, 0.1, 0.1));
  movements[4].pos_movement = Vector3d(6.0, -5.0, 2.0);
  movements[4].tan_rotation = Eigen::AngleAxisd (M_PI/90.0, Vector3d(1.0, 0.1, -0.1));


  intermediateThread.resize(0);
  Thread* curr = new Thread(start);
  Thread* next;
  for (int i=0; i < movements.size(); i++)
  {
    next = movements[i].applyMotion(curr); 

    intermediateThread.push_back(next);
    curr = next;
  }
*/

}



void Thread_RRT::initializeSearch(RRT_Node& start, RRT_Node& goal)
{
  currNodes.resize(0);
  currNodes.push_back(new RRT_Node_And_Edge(start));
}


void Thread_RRT::getNextGoal(RRT_Node& goal, RRT_Node& goalThisIter)
{
  //first, see if we are sampling randomly, or around goal
  double randNum = ((double)rand()) / ((double)RAND_MAX);
  if (randNum < RANDOM_SAMPLE_THRESH)
  {
    //goalThisIter.copyNode(goal);   
    //return;
    
    setThisGoalNearGoal(goal, goalThisIter);
    return;



  }

  //sample random goal
  Matrix4d startTrans;
  goal.thread->getStartTransform(startTrans);

  Vector3d startPos = startTrans.corner(Eigen::TopRight,3,1);
  Vector3d startTan = startTrans.corner(Eigen::TopLeft,3,1);
  goalThisIter.thread = new Thread(goal.thread->length(), startPos, startTan);

  goalThisIter.thread->minimize_energy();
  goalThisIter.setPoints();

}

void Thread_RRT::setThisGoalNearGoal(RRT_Node& goal, RRT_Node& goalThisIter)
{
  //sample something in sphere between current point and goal
  //first, set the things we know
  Matrix4d startTrans;
  goal.thread->getStartTransform(startTrans);
  Vector3d posNewThread[2];
  Vector3d tanNewThread[2];
  double lengthNewThread[2];
  double length_thread = goal.thread->length();
  lengthNewThread[0] = lengthNewThread[1] = length_thread/2.0;
  posNewThread[0] = startTrans.corner(Eigen::TopRight,3,1);
  tanNewThread[0] = startTrans.corner(Eigen::TopLeft,3,1);
  Vector3d endPos;
  goal.thread->getWantedEndPosition(endPos);
  Vector3d endTan;
  goal.thread->getWantedEndTangent(endTan);


  //random params for initializing
	double curvatureNewThread[2] = {randomNumUnit()*RANDOM_THREAD_MAX_CURVATURE_INIT, randomNumUnit()*RANDOM_THREAD_MAX_CURVATURE_INIT};
	double torsionNewThread[2] = {randomNumUnit()*RANDOM_THREAD_MAX_TORSION_INIT, randomNumUnit()*RANDOM_THREAD_MAX_TORSION_INIT};

  //find closest thread to goal
  int indClosest;
  findClosestNode(goal, indClosest);

  Vector3d endPosClosestToGoal;
  currNodes[indClosest]->this_node.thread->getWantedEndPosition(endPosClosestToGoal);
  Vector3d endTanClosestToGoal;
  currNodes[indClosest]->this_node.thread->getWantedEndTangent(endTanClosestToGoal);

  //calculate the distance to this end
  //sample from this radius, add to end point
  double randX, randY, randZ, sampledRadius;
  double radius_pos = (endPos - endPosClosestToGoal).norm() + ADD_TO_GOAL_DIST_RADIUS;
  double dist_new_thread;
  do 
  {
    //find point in sphere with this radius, add to goal endpoint
    //std::cout << "sampling end point" << std::endl;
    double sampledRadius;
    do
    {
      //std::cout << "sampling circle" << std::endl;
      randX = randomMaxAbsValue(radius_pos);
      randY = randomMaxAbsValue(radius_pos);
      randZ = randomMaxAbsValue(radius_pos);
      sampledRadius = randX*randX + randY*randY + randZ*randZ;
    } while (sampledRadius >= (radius_pos*radius_pos));
    posNewThread[1](0) = randX+endPos(0);
    posNewThread[1](1) = randY+endPos(1);
    posNewThread[1](2) = randZ+endPos(2);

    dist_new_thread = (posNewThread[1]-posNewThread[0]).norm();
  } while (dist_new_thread >= length_thread);

  //sample from a tan around the goal tan
  //sample from outside of sphere with radius equal to length between goal tan and closest tan
  //normalize the length of that new tan
  double radius_tan = (endTan - endTanClosestToGoal).norm() + ADD_TO_GOAL_TAN_RADIUS;
  double randTheta = M_PI*randomNumUnit();
  double randPhi = 2.0*M_PI*randomNumUnit();
  tanNewThread[1](0) = endTan(0)+radius_tan*cos(randTheta)*sin(randPhi);
  tanNewThread[1](1) = endTan(1)+radius_tan*sin(randTheta)*sin(randPhi);
  tanNewThread[1](2) = endTan(2)+radius_tan*cos(randPhi);

  tanNewThread[1].normalize();

//  std::cout << "goal end pos: " << endPos << std::endl;
//  std::cout << "goal end tan: " << endTan << std::endl;

//  std::cout << "new end pos: " << posNewThread[1] << std::endl;
//  std::cout << "new end tan: " << tanNewThread[1] << std::endl;

  
  goalThisIter.thread = new Thread(goal.thread);
  goalThisIter.thread->setEndConstraint(posNewThread[1], tanNewThread[1]);
  goalThisIter.thread->upsampleAndOptimize_minLength(0.065);
  goalThisIter.thread->minimize_energy();


  //goalThisIter.thread = new Thread(curvatureNewThread, torsionNewThread, lengthNewThread, 2, posNewThread, tanNewThread);
  //goalThisIter.thread = new Thread(length_thread, posNewThread[0], tanNewThread[0]);
  //goalThisIter.thread->minimize_energy();
  goalThisIter.setPoints();

  double distToGoal = goal.distanceToNode(goalThisIter);
  //std::cout << "distance to actual goal: " << distToGoal << std::endl;

}



double Thread_RRT::findClosestNode(RRT_Node& goalNode, int& ind_to_closest)
{
  ind_to_closest = 0;
  double dist = goalNode.distanceToNode(currNodes[0]->this_node);
  for (int i=1; i < currNodes.size(); i++)
  {
    double thisDist = goalNode.distanceToNode(currNodes[i]->this_node);
    if (thisDist < dist)
    {
      dist = thisDist;
      ind_to_closest = i;
    }
  }

  return dist;
}

void Thread_RRT::findThreadMotion(RRT_Node& start, RRT_Node& goal, Thread_Motion& bestMotion)
{
  /*motion.pos_movement.setZero();
  motion.tan_rotation.setIdentity();

  double estimated_derivs[5];
  Matrix4d startTrans;
  start.thread->getStartTransform(startTrans);
  
  for (int dir=0; dir < 3; dir++)
  {
    //move in direction
    motion.pos_movement(dir) = DISTANCE_TO_TRY;
    RRT_Node thisNodeToTry(motion.applyMotion(start.thread));
    double pos_dir_dist = goal.distanceToNode(thisNodeToTry);

    //move in other direction
    motion.pos_movement(dir) = -DISTANCE_TO_TRY;
    RRT_Node thisNodeToTryOtherDir(motion.applyMotion(start.thread));
    double neg_dir_dist = goal.distanceToNode(thisNodeToTryOtherDir);
    //double neg_dir_dist = goal.distanceToNode(start);

    estimated_derivs[dir] = (pos_dir_dist-neg_dir_dist)/(2.0*DISTANCE_TO_TRY); 

    //cleanup for next iter
    motion.pos_movement(dir) = 0.0;
  }
  std::cout << estimated_derivs[0] << " " << estimated_derivs[1] << " " << estimated_derivs[2] << std::endl;
  motion.pos_movement = Vector3d(estimated_derivs[0]/100.0, estimated_derivs[1]/100.0, estimated_derivs[2]/100.0);
*/




  /*
  Matrix4d startTrans;
  start.thread->getStartTransform(startTrans);
  double bestScore = DBL_MAX;
  Thread_Motion thisMotion;
  for (int randSampleInd=0; randSampleInd < NUM_RANDOM_SAMPLES; randSampleInd++)
  {
    //random position movement
    for (int i=0; i < 3; i++)
    {
      thisMotion.pos_movement(i) = (2.0*MAX_DISTANCE_MOVE_EACH_DIR*((double)rand()) / ((double)RAND_MAX))-MAX_DISTANCE_MOVE_EACH_DIR;
    }

    //random rotation matrix
    double rotAng1 = (2.0*MAX_ANG_TO_ROTATE*((double)rand()) / ((double)RAND_MAX)) - MAX_ANG_TO_ROTATE;
    double rotAng2 = (2.0*MAX_ANG_TO_ROTATE*((double)rand()) / ((double)RAND_MAX)) - MAX_ANG_TO_ROTATE;
    thisMotion.setRotationMatrixFromAngs(rotAng1, rotAng2);
   
    //std::cout << "pos: " << thisMotion.pos_movement << std::endl;
    //std::cout << "tan: " << thisMotion.tan_rotation << std::endl;

    RRT_Node thisMotionTried(thisMotion.applyMotion(start.thread));
    double thisScore = goal.distanceToNode(thisMotionTried);

    if (thisScore < bestScore)
    {
      bestScore = thisScore;
      bestMotion = thisMotion;
    }
    
  }
  */

  
  //find vector to get position closer
  Vector3d startEnd;
  start.thread->getWantedEndPosition(startEnd);
  Vector3d goalEnd;
  goal.thread->getWantedEndPosition(goalEnd);

  bestMotion.pos_movement = goalEnd - startEnd;

  //add some noise to movement
  bestMotion.pos_movement(0) += randomMaxAbsValue(MAX_NOISE_DISTANCE_MOVEMENT);
  bestMotion.pos_movement(1) += randomMaxAbsValue(MAX_NOISE_DISTANCE_MOVEMENT);
  bestMotion.pos_movement(2) += randomMaxAbsValue(MAX_NOISE_DISTANCE_MOVEMENT);

  //make sure vector isn't too long (that's what she said)
  double normOfVec = bestMotion.pos_movement.norm();
  if (normOfVec > MAX_DISTANCE_MOVEMENT)
  {
    bestMotion.pos_movement *= (MAX_DISTANCE_MOVEMENT/normOfVec);
  }
 
  //rotate towards other tan
  Vector3d startTan;
  start.thread->getWantedEndTangent(startTan);
  Vector3d goalTan;
  goal.thread->getWantedEndTangent(goalTan);

  Vector3d vecToRotateAbout = startTan.cross(goalTan);
  vecToRotateAbout.normalize();
  //add noise to tan
  vecToRotateAbout(0) += randomMaxAbsValue(MAX_NOISE_ROTATION_VECTOR);
  vecToRotateAbout(1) += randomMaxAbsValue(MAX_NOISE_ROTATION_VECTOR);
  vecToRotateAbout(2) += randomMaxAbsValue(MAX_NOISE_ROTATION_VECTOR);
  vecToRotateAbout.normalize();

  double angToRotate = min(MAX_ANG_TO_ROTATE, acos(startTan.dot(goalTan)));
  angToRotate += randomMaxAbsValue(MAX_NOISE_ROTATION_ANGLE);

  bestMotion.tan_rotation = Eigen::AngleAxisd( angToRotate, vecToRotateAbout);

  

  //motion.pos_movement = Vector3d(4.0, -3.5, 1.0);
  //motion.tan_rotation = Eigen::AngleAxisd (0.0, Vector3d(1.0, 0.0, 0.0));

}

void Thread_RRT::cleanup()
{
  for (int i=0; i < currNodes.size(); i++)
  {
    delete currNodes[i];
  }
  currNodes.resize(0);
}














/****************************************************************************
 ******************************* RRT_Node ***********************************
****************************************************************************/


RRT_Node::RRT_Node(const Thread* threadIn) :
 thread(new Thread(threadIn))
{
  setPoints();
}

RRT_Node::RRT_Node(const RRT_Node& nodeToCopy) :
  points(nodeToCopy.points)
{
  if (nodeToCopy.thread != NULL)
    thread = new Thread(nodeToCopy.thread);
  else
    thread = NULL;
}


RRT_Node::~RRT_Node()
{
  if (thread != NULL)
    delete thread;
}

void RRT_Node::setPoints()
{
  //points = MatrixXd::Zero(NUM_PTS_PER_NODE, 3);
  points.resize(NUM_PTS_PER_NODE,3);
  thread->getPoints(points);
}

void RRT_Node::copyNode(const RRT_Node& toCopy)
{
  if (thread != NULL)
    delete thread;
  thread = new Thread(toCopy.thread);
  points = toCopy.points;
}

double RRT_Node::distanceToPoints(MatrixXd& otherPoints)
{
  return avgDistBetweenPoints(points, otherPoints);
}

double RRT_Node::distanceToNode(RRT_Node& otherNode)
{
  return avgDistBetweenPoints(points, otherNode.points);
  //return distanceToPoints(otherNode.points);
}


void RRT_Node::applyMotion(RRT_Node& end, Thread_Motion& motion)
{
  end.thread = motion.applyMotion(thread);
  end.setPoints();
}


RRT_Node& RRT_Node::operator=(const RRT_Node& rhs)
{
  copyNode(rhs);
}











/****************************************************************************
 **************************** RRT_Node_And_Edge *****************************
****************************************************************************/

RRT_Node_And_Edge::~RRT_Node_And_Edge()
{
  if (motion_from_last != NULL)
    delete motion_from_last;
}

RRT_Node_And_Edge& RRT_Node_And_Edge::operator=(const RRT_Node_And_Edge& rhs)
{
  this_node = rhs.this_node;
  last_node = rhs.last_node;
  motion_from_last = rhs.motion_from_last;
}



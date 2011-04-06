#include "rrt_utils.h"
#include <time.h>

RRTNode::~RRTNode() {
  for(int i = 0; i < lstMotions.size(); i++) {
    delete lstMotions[i];
  }
  lstMotions.clear();

  delete data; 

  //if (thread != NULL) { delete thread; }

}

RRTNode::RRTNode(const Thread* start): prev(NULL), next(NULL), linearized(false) {
//  start->toVector(&x);
//  start->getTwists(&twists);
//  endrot = start->end_rot();
  thread = (Thread *) start;   
  //B.resize(start->num_pieces()*3, 6);
  CVF = 0.0; 

  VectorXd positions;
  start->toVector(&positions);
  
  VectorXd twists;
  start->getTwists(&twists); 

  VectorXd gradient;
  start->getEdges(&gradient); 

  VectorXd curvature_binormal;
  start->getCurvatureBinormal(&curvature_binormal); 


  

  N = positions.size() + twists.size() + 
    gradient.size() + curvature_binormal.size();
  data = new float[N];
  
  int currentIndex = 0; 

  for (int i = 0; i < positions.size(); i++) {
    if (!isnan(positions[i]))
        data[i + currentIndex] = (float) (10*positions[i]); 
  }
  
  currentIndex += positions.size(); 
  for (int i = 0; i < twists.size(); i++) {
    if (!isnan(twists[i]))  
        data[i+currentIndex] = (float) (5*twists[i]);
  }

  currentIndex += twists.size(); 
  for (int i = 0; i < gradient.size(); i++) {
    if (!isnan(gradient[i])) 
      data[i+currentIndex] = (float) (10*gradient[i]);
  }

  currentIndex += gradient.size(); 
  for (int i = 0; i < curvature_binormal.size(); i++) {
    if(!isnan(curvature_binormal[i])) 
        data[i+currentIndex] = (float) (0*curvature_binormal[i]);
  }


}
  


double RRTNodeUtils::distanceBetween(RRTNode* start, RRTNode* end) { 
  int N = start->N;

  const float* startData = start->getData();
  const float* endData = end->getData();


  float r = 0.0;
  for (int i = 0; i < N; i++) { 
    r += (startData[i] - endData[i]) * (startData[i] - endData[i]);  
  }

  return r; 
}


double RRTNodeUtils::l2PointsDifference(RRTNode* start, RRTNode* end) {
  VectorXd startPosition;
  VectorXd endPosition; 

  start->thread->toVector(&startPosition);
  end->thread->toVector(&endPosition);

  return (startPosition - endPosition).norm(); 


}



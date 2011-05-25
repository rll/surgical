#include "rrt_utils.h"
#include <time.h>

RRTNode::~RRTNode() {
  delete[] data; 

  //if (thread != NULL) { delete thread; }
}

RRTNode::RRTNode(const Thread* start): prev(NULL), next(NULL), linearized(false) {
  
  thread = (Thread *) start;   
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
    gradient.size();
  data = new float[N];
  
  int currentIndex = 0; 

  for (int i = 0; i < positions.size(); i++) {
    if (!isnan(positions[i])) { 
        data[i + currentIndex] = 5*((float)positions[i]);
    } else {
        cout << "Warning: Position NaN; Setting to 0" << endl;
        data[i + currentIndex] = 0;
    }
  }
  
  currentIndex += positions.size(); 
  for (int i = 0; i < twists.size(); i++) {
    if (!isnan(twists[i])) {   
        data[i+currentIndex] = 5*((float)twists[i]);
    } else { 
      cout << "Warning: Twist NaN; Setting to 0" << endl; 
      data[i+currentIndex] = 0; 
    }
  }

  currentIndex += twists.size(); 
  for (int i = 0; i < gradient.size(); i++) {
    if (!isnan(gradient[i])) { 
      data[i+currentIndex] = 5*((float)gradient[i]);
    } else { 
      cout << "Warning: Gradient NaN; Setting to 0" << endl; 
      data[i + currentIndex] = 0; 
    }
  }

  /*currentIndex += gradient.size(); 
  for (int i = 0; i < curvature_binormal.size(); i++) {
    if(!isnan(curvature_binormal[i])) 
        data[i+currentIndex] = 0; //(float) (0*curvature_binormal[i]);
  }*/
}
  
double RRTNodeUtils::distanceBetween(RRTNode* start, RRTNode* end) { 
  int N = start->N;
  const float* startData = start->getData();
  const float* endData = end->getData();
  double r = 0.0;
  for (int i = 0; i < N; i++) { 
    //cout << "Start: " << startData[i] << endl;
    //cout << "End:" << endData[i] << endl;
    r += pow((startData[i] - endData[i]), 2);  
    //cout << "R = " << r << endl; 
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



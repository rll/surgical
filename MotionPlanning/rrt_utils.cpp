#include "rrt_utils.h"
#include <time.h>

RRTNode::~RRTNode() {
  for(int i = 0; i < lstMotions.size(); i++) {
    delete lstMotions[i];
  }
  lstMotions.clear();
  //if (thread != NULL) { delete thread; }

}

RRTNode::RRTNode(const Thread* start): prev(NULL), next(NULL), linearized(false) {
//  start->toVector(&x);
//  start->getTwists(&twists);
//  endrot = start->end_rot();
  thread = (Thread *) start;   
  //B.resize(start->num_pieces()*3, 6);
  CVF = 0.0; 
  N = start->num_pieces()*3;
  data = new float[N];
  for (int i = 0; i < start->num_pieces(); i++) {
    Vector3d v = start->vertex_at_ind(i);
    for (int j = 0; j < 3; j++) {
      data[3*i + j] = (float) v[j];
    }
  }
  
}

/*RRTNode::Accessor::Accessor() { 
  nodes = new set<RRTNode*>();
}
*/



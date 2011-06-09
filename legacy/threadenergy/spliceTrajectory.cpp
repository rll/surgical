#include <stdlib.h>
#include <iostream>
#include <fstream>

#ifdef MAC
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <GL/gle.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h>
#endif


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include "thread_minenergy.h"
#include "trajectory_recorder.h"
#include "trajectory_reader.h"
#include "globals_thread_param_estimation.h"

USING_PART_OF_NAMESPACE_EIGEN

int main(int argc, char* argv[]) {
  if(argc < 2) {
    cout << "usage: \"spliceTrajectory <setname>\" where <setname> is relative to the ../vision/suture_sets/ directory" << endl;
    exit(1);
  }

  char motionsFileName[256];
  sprintf(motionsFileName,"../vision/suture_sets/%s/%s_traj.txt.input", argv[1], argv[1]);
  char threadsFileName[256];
  sprintf(threadsFileName,"../vision/suture_sets/%s/%s_traj_imgs.txt.input", argv[1], argv[1]);
  char pointsFileName[256];
  sprintf(pointsFileName,"../vision/suture_sets/%s/%s_traj_imgs.points", argv[1], argv[1]);

  cout <<"Reading files: " << motionsFileName << " " << threadsFileName << " " << pointsFileName << endl;
  Trajectory_Reader traj_reader(motionsFileName, threadsFileName, pointsFileName);
  traj_reader.read_threads_from_file();

  //char spliceFileName
  char spliceFileName[256];
  sprintf(spliceFileName,"../vision/suture_sets/%s/splice", argv[1]);

  vector<int> toDelete;
  ifstream test(spliceFileName);
  int numToDelete;
  int read;
  test >> numToDelete;
  for(int i = 0; i < numToDelete; i++) {
    test >> read;
    toDelete.push_back(read);
  }
  for(int i = 0; i < numToDelete; i++) {
    cout << toDelete[i] << " " << endl;
  }
  test.close();



  vector<savedThread>* threads = traj_reader.get_all_points();
  if(threads->size()==0) {
    cout << "error, no threads read" << endl;
    exit(1);
  }

  char motionsOutFileName[256];
  sprintf(motionsOutFileName,"../vision/suture_sets/%s/%s_traj.txt", argv[1], argv[1]);
  char threadsOutFileName[256];
  sprintf(threadsOutFileName,"../vision/suture_sets/%s/%s_traj_imgs", argv[1], argv[1]);

  cout <<"Writing files: " << motionsOutFileName << " " << threadsOutFileName << endl;

  Trajectory_Recorder myrecord(motionsOutFileName, threadsOutFileName);

  myrecord.setLength(traj_reader.length());
  savedThread last_thread;
  savedThread cur_thread;

  int lastind = -1;


  vector<int>::iterator result;

  for(int i = 0; i < threads->size(); i++) {
    result = find(toDelete.begin(), toDelete.end(), i);
    if(result != toDelete.end()) {// did not contain this
      last_thread = threads->at(i);
      myrecord.add_thread_to_list(last_thread.points, last_thread.positions, last_thread.tangents);
      lastind = i;
      break;
    }
  }


  for(int i = lastind+1; i < threads->size(); i++) {
    result = find(toDelete.begin(), toDelete.end(), i);
    if(result!=toDelete.end()) {
      cur_thread = threads->at(i);
      myrecord.add_thread_to_list(cur_thread.points, cur_thread.positions, cur_thread.tangents);
      myrecord.add_motion_to_list(last_thread.positions[1], cur_thread.positions[1], last_thread.tangents[1], cur_thread.tangents[1]);
      last_thread = cur_thread;
    }
  }

  myrecord.write_threads_to_file();
  myrecord.write_motions_to_file();
  return 0;
}

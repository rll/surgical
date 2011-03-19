#include "trajectory_reader.h"


Trajectory_Reader::Trajectory_Reader()
{
 strcpy(_fileName_motions, TRAJ_FILE_PATH);
 sprintf(_fileName_threads, "%s.txt", IMAGE_BASE_NAME);
 sprintf(_fileName_thread_points, "%s.points", IMAGE_BASE_NAME);
 _currInd_motions = 0;
 _currInd_points = 0;
 _start_thread = NULL;
}

Trajectory_Reader::Trajectory_Reader(char* fileName_motions_in, char* fileName_threads_in, char* fileName_points_in)
{
 strcpy(_fileName_motions, fileName_motions_in);
 strcpy(_fileName_threads, fileName_threads_in);
 sprintf(_fileName_thread_points, fileName_points_in);
 _currInd_motions = 0;
 _currInd_points = 0;
 _start_thread = NULL;
}


void Trajectory_Reader::read_motions_from_file()
{
  ifstream trajectory_playback;
  trajectory_playback.open(_fileName_motions);
  while (!trajectory_playback.eof())
  {
    _motions.resize(_motions.size()+1);
    trajectory_playback >> _motions.back().pos_movement(0) >> _motions.back().pos_movement(1) >> _motions.back().pos_movement(2);
    trajectory_playback >> _motions.back().tan_rotation(0,0) >> _motions.back().tan_rotation(0,1) >> _motions.back().tan_rotation(0,2);
    trajectory_playback >> _motions.back().tan_rotation(1,0) >> _motions.back().tan_rotation(1,1) >> _motions.back().tan_rotation(1,2);
    trajectory_playback >> _motions.back().tan_rotation(2,0) >> _motions.back().tan_rotation(2,1) >> _motions.back().tan_rotation(2,2);
    std::cout << _motions.back().pos_movement(0) << std::endl;
  }

  if (_motions.size() > 0)
    _motions.resize(_motions.size()-1);
}





void Trajectory_Reader::read_threads_from_file()
{
  ifstream threads_playback;
  threads_playback.open(_fileName_threads);
//  threads_playback.precision(10);
  int num_pts_each;
  threads_playback >> _length_thread >> num_pts_each;

  // check to see if thread data exists, set a flag
  ifstream thread_point_playback(_fileName_thread_points);
  ofstream thread_point_writer;
  if(thread_point_playback) {
    cout << "found file, loading from file" << endl;
  } else {
    cout << "didn't find file, computing each string point" << endl;
    thread_point_writer.open(_fileName_thread_points);
    thread_point_writer.precision(10);
  }


  MatrixXd toAdd(num_pts_each, 3);
  while (!threads_playback.eof())
  {
    for (int i=0; i < num_pts_each; i++)
    {
      threads_playback >> toAdd(i,0) >> toAdd(i,1) >> toAdd(i,2);
    }
    Vector3d positions[2];
    Vector3d tangents[2];

    threads_playback >> positions[0][0] >> positions[0][1] >> positions[0][2];
    threads_playback >> positions[1][0] >> positions[1][1] >> positions[1][2];
    threads_playback >> tangents[0][0] >> tangents[0][1] >> tangents[0][2];
    threads_playback >> tangents[1][0] >> tangents[1][1] >> tangents[1][2];

    tangents[0].normalize();
    tangents[1].normalize();


    double curvatures[2];
    double torsions[2];
    double lengths[2];
    curvatures[0] = 2.0;
    curvatures[1] = 1.0;
    torsions[0] = 1.0;
    torsions[1] = 0.5;
    lengths[0] = lengths[1] = _length_thread/2.0;
    if(_start_thread) { // this is happening in a weird place so we miss writing out the last weird thread
 //     _start_thread->printThreadInfo();
      _start_thread->toStream(thread_point_writer);
      delete _start_thread;
    }
    _start_thread = new Thread(curvatures, torsions, lengths, 2, positions, tangents);

    // either read point data in and pass it into savedthread constructor
    if(thread_point_playback) {
      _start_thread->fromStream(thread_point_playback);
      // read start_thread from file, add a copy of it to _points_each_thread
    } else {
      // or compute it here
      _start_thread->optimizeManyPoints_startAndEnd_MyParams(toAdd, 2, positions, tangents);
    }

    savedThread readFromFile(toAdd, positions, tangents);
    _points_each_thread.push_back(readFromFile);

    Thread* threadptr = new Thread(_start_thread);
    _ptr_each_thread.push_back(threadptr);
  }

  if (_points_each_thread.size() > 0) {
    _points_each_thread.resize(_points_each_thread.size()-1);
    _ptr_each_thread.resize(_ptr_each_thread.size()-1);
  }


}

void Trajectory_Reader::estimate_init_thread()
{
  double curvatures[2];
  double torsions[2];
  double lengths[2];
  curvatures[0] = 2.0;
  curvatures[1] = 1.0;
  torsions[0] = 1.0;
  torsions[1] = 0.5;
  lengths[0] = lengths[1] = _length_thread/2.0;

  _start_thread = new Thread(curvatures, torsions, lengths, 2, _points_each_thread.front().positions, _points_each_thread.front().tangents);
  _start_thread->optimizeManyPoints_startAndEnd_MyParams(_points_each_thread.front().points, 2, _points_each_thread.front().positions, _points_each_thread.front().tangents);
}


bool Trajectory_Reader::get_next_motion(Thread_Motion& nextMotion)
{
  if (_currInd_motions >= _motions.size())
    return false;

  nextMotion = _motions[_currInd_motions];
  _currInd_motions++;
  return true;
}




#include "trajectory_reader.h"


Trajectory_Reader::Trajectory_Reader()
{
 strcpy(_fileName_motions, TRAJ_FILE_PATH);
 sprintf(_fileName_threads, "%s.txt", IMAGE_BASE_NAME);
 _currInd_motions = 0;
 _currInd_points = 0;
}

Trajectory_Reader::Trajectory_Reader(char* fileName_motions_in, char* fileName_threads_in)
{
 strcpy(_fileName_motions, fileName_motions_in);
 strcpy(_fileName_threads, fileName_threads_in);
 _currInd_motions = 0;
 _currInd_points = 0;
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


  

  MatrixXd toAdd(num_pts_each, 3);
  while (!threads_playback.eof())
  {
    for (int i=0; i < num_pts_each; i++)
    {
      threads_playback >> toAdd(i,0) >> toAdd(i,1) >> toAdd(i,2); 
    }
    _points_each_thread.push_back(toAdd);
  }

  if (_points_each_thread.size() > 0)
    _points_each_thread.resize(_points_each_thread.size()-1);

}

void Trajectory_Reader::estimate_init_thread()
{
  _start_thread = new Thread(_points_each_thread[0], 2, _length_thread);
}


bool Trajectory_Reader::get_next_motion(Thread_Motion& nextMotion)
{
  if (_currInd_motions >= _motions.size())
    return false;

  nextMotion = _motions[_currInd_motions];
  _currInd_motions++;
  return true;
}




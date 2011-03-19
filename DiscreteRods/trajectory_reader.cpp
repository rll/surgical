#include "trajectory_reader.h"


Trajectory_Reader::Trajectory_Reader()
{
 sprintf(_fileName_threads, "%s.txt", THREADS_BASE_NAME);
}

Trajectory_Reader::Trajectory_Reader(const char* fileName_threads_in)
{
 sprintf(_fileName_threads, "%s.txt", fileName_threads_in);
}

void Trajectory_Reader::set_file(const char* fileName_threads_in)
{
 sprintf(_fileName_threads, "%s.txt", fileName_threads_in);
}

void Trajectory_Reader::read_threads_from_file()
{
 std::cout << "filename: " << _fileName_threads << std::endl;
  ifstream threads_playback;
  threads_playback.open(_fileName_threads);
//  threads_playback.precision(10);
  int num_pts_each;
  int read;
  threads_playback >> num_pts_each;
 // threads_playback >> num_pts_each;

  vector<Vector3d> points;
  points.resize(num_pts_each);
  vector<double> twist_angles;
  twist_angles.resize(num_pts_each);
  Matrix3d start_rot;
  Matrix3d end_rot;

  int num = 0;
  while (!threads_playback.eof())
  {
    
    for (int r=0; r < 3; r++)
    {
      for (int c=0; c < 3; c++)
      {
        threads_playback >> start_rot (r,c);
      }
    }

    for (int r=0; r < 3; r++)
    {
      for (int c=0; c < 3; c++)
      {
        threads_playback >> end_rot (r,c);
      }
    }


    for (int i=0; i < points.size(); i++)
    {
      threads_playback >> points[i](0) >> points[i](1) >> points[i](2) >> twist_angles[i];
    }
		twist_angles.back() = twist_angles[twist_angles.size()-2];

    Thread nextThread(points, twist_angles, start_rot);
    _each_thread.push_back(nextThread);
		//_each_thread.back() = nextThread;
  }


  //last read thread is garbage data
  if (_each_thread.size() > 0)
    _each_thread.pop_back();

}



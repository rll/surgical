#include "controls_reader.h"


Controls_Reader::Controls_Reader(const char* fileName_controls_in)
{
 sprintf(_fileName_threads, "%s.txt", fileName_controls_in);
}


void Controls_Reader::read_controls_from_file()
{
  std::cout << "filename: " << _fileName_controls_in << std::endl;


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

    Thread nextThread(points, twist_angles, start_rot, DEFAULT_REST_LENGTH);
    _each_thread.push_back(nextThread);
		//_each_thread.back() = nextThread;

  }

  //last read thread is garbage data
  if (_each_thread.size() > 0)
    _each_thread.pop_back();

}


void Trajectory_Reader::get_all_threads(vector<Thread*>& threads_out)
{
  for (int i=0; i < _each_thread.size(); i++)
  {
    threads_out.push_back(new Thread(_each_thread[i]));
  }
}

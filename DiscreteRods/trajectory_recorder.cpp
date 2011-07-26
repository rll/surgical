#include "trajectory_recorder.h"



Trajectory_Recorder::Trajectory_Recorder()
{
    sprintf(_fileName_threads, "%s.txt", THREADS_BASE_NAME);
}

Trajectory_Recorder::Trajectory_Recorder(const char* fileName_threads_in)
{
    sprintf(_fileName_threads, "%s.txt", fileName_threads_in);
}


void Trajectory_Recorder::add_thread_to_list(const Thread& thread)
{
  _threads.push_back(thread);
	//_threads.back() = thread;
}

void Trajectory_Recorder::add_thread_to_list(const Thread* thread)
{
  _threads.push_back(*thread);
}

void Trajectory_Recorder::add_threads_to_list(vector<Thread*> threads)
{
  for (int i=0; i < threads.size(); i++)
  {
    _threads.push_back(*threads[i]);
  }
}

void Trajectory_Recorder::clear_threads()
{
	_threads.resize(0);
}

void Trajectory_Recorder::write_threads_to_file()
{

  std::cout << "Writing to: " << _fileName_threads << std::endl;

  ofstream threadFile;
  threadFile.precision(20);
  threadFile.open(_fileName_threads);

  if (_threads.size() == 0)
    return;

  vector<Vector3d> points;
  vector<double> twist_angles;
  vector<double> rest_lengths;
  Matrix3d start_rot;
  Matrix3d end_rot;
  _threads.front().get_thread_data(points, twist_angles, rest_lengths);

 threadFile << (double)points.size() << "\n";

  //write each point for each thread
  for (int i=0; i < _threads.size(); i++)
  {
    _threads[i].get_thread_data(points, twist_angles, rest_lengths);
    start_rot = _threads[i].start_rot();
    end_rot = _threads[i].end_rot();

    for (int r=0; r < 3; r++)
    {
      for (int c=0; c < 3; c++)
      {
        threadFile << start_rot (r,c) << " ";
      }
    }

    for (int r=0; r < 3; r++)
    {
      for (int c=0; c < 3; c++)
      {
        threadFile << end_rot (r,c) << " ";
      }
    }


		//std::cout << "twist angles: " << std::endl;
    for (int j=0; j < points.size(); j++)
    {
      threadFile << points[j](0) << " " << points[j](1) << " " << points[j](2) << " " << twist_angles[j] << " " << rest_lengths[j] << " ";
			//std::cout << twist_angles[j] << " ";
    }
    threadFile << "\n";
		//std::cout << "\n";

  }

  threadFile.close();

  std::cout << "Writing Done\n";
}

void Trajectory_Recorder::setFileName(const char* newFileName) {
    sprintf(_fileName_threads, "%s.txt", newFileName);
}






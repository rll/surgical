#include "TrajectoryRecorder.h"

TrajectoryRecorder::TrajectoryRecorder()
{
	sprintf(_fileName, "%s.txt", TRAJECTORY_BASE_NAME);
	started = false;
}

TrajectoryRecorder::TrajectoryRecorder(const char* fileName)
{
	sprintf(_fileName, "%s.txt", fileName);
	started = false;
}

void TrajectoryRecorder::setFileName(const char* fileName)
{
	sprintf(_fileName, "%s.txt", fileName);
	started = false;
}

void TrajectoryRecorder::start()
{
  std::cout << "Writing trajectory to: " << _fileName << std::endl;

  file.precision(20);
  file.open(_fileName);
  
  started = true;
}

void TrajectoryRecorder::stop()
{
  file << NO_STATE << " ";
  file << "\n";

  file.close();

  std::cout << "Writing trajectory Done\n";
  
	started = false;
}

void TrajectoryRecorder::writeWorldToFile(World* world)
{
  file << STATE << " ";
  world->writeToFile(file);
}

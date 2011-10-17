#include "StateRecorder.h"

StateRecorder::StateRecorder()
{
	sprintf(_fileName, "%s.txt", STATE_BASE_NAME);
}

StateRecorder::StateRecorder(const char* fileName)
{
	sprintf(_fileName, "%s.txt", fileName);
}

void StateRecorder::setFileName(const char* fileName)
{
	sprintf(_fileName, "%s.txt", fileName);
}

void StateRecorder::writeWorldToFile(World* world)
{
  std::cout << "Writing to: " << _fileName << std::endl;

  ofstream file;
  file.precision(20);
  file.open(_fileName);

  world->writeToFile(file);

  file.close();

  std::cout << "Writing Done\n";
}

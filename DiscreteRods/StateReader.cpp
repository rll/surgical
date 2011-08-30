#include "StateReader.h"

StateReader::StateReader()
{
 sprintf(_fileName, "%s.txt", STATE_BASE_NAME);
}

StateReader::StateReader(const char* fileName)
{
 sprintf(_fileName, "%s.txt", fileName);
}

void StateReader::setFileName(const char* fileName)
{
 sprintf(_fileName, "%s.txt", fileName);
}

bool StateReader::readWorldFromFile(World* world)
{
	std::cout << "filename: " << _fileName << std::endl;
  ifstream file;
  file.open(_fileName);
  
  if (file.fail()) {
  	cout << "Failed to open file. Objects were not read from file." << endl;
  	return false;
  }
  
  if (world != NULL) {
		delete world;
		world = NULL;
	}

  world = new World(file);

  file.close();
  return true;
}

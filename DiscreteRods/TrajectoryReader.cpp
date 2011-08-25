#include "TrajectoryReader.h"

TrajectoryReader::TrajectoryReader()
{
 sprintf(_fileName, "%s.txt", TRAJECTORY_BASE_NAME);
}

TrajectoryReader::TrajectoryReader(const char* fileName)
{
 sprintf(_fileName, "%s.txt", fileName);
}

void TrajectoryReader::setFileName(const char* fileName)
{
 sprintf(_fileName, "%s.txt", fileName);
}

bool TrajectoryReader::readStatesFromFile(vector<World*>& worlds)
{
  std::cout << "filename: " << _fileName << std::endl;
  ifstream file;
  file.open(_fileName);
  
  if (file.fail()) {
  	cout << "Failed to open file. Objects were not read from file." << endl;
  	return false;
  }
  
  int state;
  bool success = true;
  for (int i=0; i<worlds.size(); i++) {
  	delete worlds[i];
  	worlds[i] = NULL;
  }
  worlds.clear();
  
  while (!file.eof()) {
    file >> state;
    if (state == STATE) {
      World* world = new World();
      StateReader state_reader(_fileName);
      success = state_reader.readFromFile(file, world) && success;
      if (!success)
        break;
      worlds.push_back(world);
    } else {
      break;
    }
  }
  
  return success;
}

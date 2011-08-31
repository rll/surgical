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

//the contents of worlds should be properly formatted, i.e. if the world object has already been deleted, then it should be NULL
bool TrajectoryReader::readWorldsFromFile(vector<World*>& worlds)
{
  std::cout << "filename: " << _fileName << std::endl;
  ifstream file;
  file.open(_fileName);
  
  if (file.fail()) {
  	cout << "Failed to open file. Objects were not read from file." << endl;
  	return false;
  }
  
  int state;
  for (int i=0; i<worlds.size(); i++) {
  	if (worlds[i] != NULL) {
  		delete worlds[i];
  		worlds[i] = NULL;
  	}
  }
  worlds.clear();
  
  while (!file.eof()) {
    file >> state;
    if (state == STATE) {
      worlds.push_back(new World(file));
    } else {
      break;
    }
  }
  
  file.close();
  
  return true;
}

bool TrajectoryReader::readControlsFromFile(vector<Control*>& controls)
{
  std::cout << "filename: " << _fileName << std::endl;
  ifstream file;
  file.open(_fileName);
  
  if (file.fail()) {
  	cout << "Failed to open file. Controls were not read from file." << endl;
  	return false;
  }
  
  int type;
  for (int i=0; i<controls.size(); i++) {
  	if (controls[i] != NULL) {
  		delete controls[i];
  		controls[i] = NULL;
  	}
  }
  controls.clear();
  
  while (!file.eof()) {
    file >> type;
    if (type == CONTROL) {
      controls.push_back(new Control(file));
    } else {
      break;
    }
  }
  
  file.close();
  
  return true;
}

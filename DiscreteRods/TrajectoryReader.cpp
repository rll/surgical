#include "TrajectoryReader.h"

TrajectoryReader::TrajectoryReader(const char* new_base_name)
{
	strcpy(base_name, new_base_name);
}

TrajectoryReader::TrajectoryReader(const char* new_file_name, const char* new_base_name)
{
	strcpy(file_name, new_file_name);
	strcpy(base_name, new_base_name);
}

void TrajectoryReader::setBaseName(const char* new_base_name)
{
	strcpy(base_name, new_base_name);
}

void TrajectoryReader::setFileName(const char* new_file_name)
{
	strcpy(file_name, new_file_name);
}

void TrajectoryReader::getFileName(char* name)
{
	strcpy(name, file_name);
}

void TrajectoryReader::queryFileName()
{
	cout << "Please enter source file name for trajectory (with extension): ";
	cin >> file_name;
}

void TrajectoryReader::extension(char* ext, char* full_path)
{
  string full_path_string(full_path);
  vector<string> vect;
  boost::split(vect, full_path_string, boost::is_any_of("."));

  if (vect.size() == 1) {
    strcat(full_path, ".txt");    
    strcpy(ext, "txt");
  } else {
    strcpy(ext, vect.back().c_str());
  }
}

StateType TrajectoryReader::trajectoryType()
{
  char *full_path = new char[256];
  sprintf(full_path, "%s%s", base_name, file_name);
  char *ext = new char[256];
  extension(ext, full_path);
  if (strcmp(ext, "tcl")) {
    return CONTROL;
  } else {  // twd or txt
    return STATE;
  }
}

//the contents of worlds should be properly formatted, i.e. if the world object has already been deleted, then it should be NULL
bool TrajectoryReader::readWorldsFromFile(vector<World*>& worlds)
{
  char *full_path = new char[256];
  sprintf(full_path, "%s%s", base_name, file_name);
  char *ext = new char[256];
  extension(ext, full_path);

  cout << "Reading world trajectory from: " << full_path << endl;
  ifstream file;
  file.open(full_path);
  
  if (file.fail()) {
  	cout << "Failed to open file. Trajectory was not loaded. Specified file might not exist." << endl;
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
  cout << "Trajectory loading was sucessful. " << worlds.size() << " worlds were loaded." << endl;
  return true;
}

bool TrajectoryReader::readControlsFromFile(vector<vector<Control*> >& controls)
{
  char *full_path = new char[256];
  sprintf(full_path, "%s%s", base_name, file_name);
  char *ext = new char[256];
  extension(ext, full_path);

  cout << "Reading control trajectory from: " << full_path << endl;
  ifstream file;
  file.open(full_path);
  
  if (file.fail()) {
  	cout << "Failed to open file. Trajectory was not loaded. Specified file might not exist." << endl;
  	return false;
  }
  
  int type;
  for (int i=0; i<controls.size(); i++) {
    for (int j=0; j<controls[i].size(); j++) {
    	if (controls[i][j] != NULL) {
    		delete controls[i][j];
    		controls[i][j] = NULL;
    	}
    }
    controls[i].clear();
  }
  controls.clear();
  
  while (!file.eof()) {
    file >> type;
    if (type == CONTROL) {
      vector<Control*> control_pair;
      control_pair.push_back(new Control(file));
      control_pair.push_back(new Control(file));
      controls.push_back(control_pair);
    } else {
      break;
    }
  }
  
  file.close();
  cout << "Trajectory loading was sucessful. " << controls.size() << " control pairs were loaded." << endl;
  return true;
}

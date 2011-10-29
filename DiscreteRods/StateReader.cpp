#include "StateReader.h"

StateReader::StateReader(const char* new_base_name)
{
	strcpy(base_name, new_base_name);
}

StateReader::StateReader(const char* new_file_name, const char* new_base_name)
{
	strcpy(file_name, new_file_name);
	strcpy(base_name, new_base_name);
}

void StateReader::setBaseName(const char* new_base_name)
{
	strcpy(base_name, new_base_name);
}

void StateReader::setFileName(const char* new_file_name)
{
	strcpy(file_name, new_file_name);
}

void StateReader::getFileName(char* name)
{
	strcpy(name, file_name);
}

void StateReader::queryFileName()
{
	cout << "Please enter source file name for state (with extension): ";
	cin >> file_name;
}

void StateReader::extension(char* ext, char* full_path)
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

bool StateReader::readWorldFromFile(World*& world)
{
	char *full_path = new char[256];
  sprintf(full_path, "%s%s", base_name, file_name);
  char *ext = new char[256];
  extension(ext, full_path);

  cout << "Reading world trajectory from: " << full_path << endl;
  ifstream file;
  file.open(full_path);
    
  if (file.fail()) {
  	cout << "Failed to open file. State was not loaded. Specified file might not exist." << endl;
  	return false;
  }
  
  if (world != NULL) {
		delete world;
		world = NULL;
	}

  world = new World(file);

  file.close();
  cout << "State loading was sucessful." << endl;
  return true;
}

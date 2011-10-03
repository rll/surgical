#include "StateRecorder.h"

StateRecorder::StateRecorder(const char* new_base_name)
{
	strcpy(base_name, new_base_name);
}

StateRecorder::StateRecorder(const char* new_file_name, const char* new_base_name)
{
	strcpy(file_name, new_file_name);
	strcpy(base_name, new_base_name);
}

void StateRecorder::setBaseName(const char* new_base_name)
{
	strcpy(base_name, new_base_name);
}

void StateRecorder::setFileName(const char* new_file_name)
{
	strcpy(file_name, new_file_name);
}

void StateRecorder::getFileName(char* name)
{
	strcpy(name, file_name);
}

void StateRecorder::queryFileName()
{
	cout << "Please enter destination file name for state (without extension): ";
	cin >> file_name;
}

void StateRecorder::writeWorldToFile(World* world)
{
	char *full_path = new char[256];
	sprintf(full_path, "%s%s%s", base_name, file_name, ".swd");
	cout << "Writing world state to: " << full_path << endl;

  ofstream file;
  file.precision(20);
  file.open(full_path);

  world->writeToFile(file);

  file.close();

  cout << "Writing trajectory to " << full_path << " is done" << endl;
}

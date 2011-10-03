#include "TrajectoryRecorder.h"

TrajectoryRecorder::TrajectoryRecorder(const char* new_base_name)
{
	strcpy(base_name, new_base_name);
	state_type = NO_STATE;
}

TrajectoryRecorder::TrajectoryRecorder(const char* new_file_name, const char* new_base_name)
{
	strcpy(file_name, new_file_name);
	strcpy(base_name, new_base_name);
	state_type = NO_STATE;
}

void TrajectoryRecorder::setBaseName(const char* new_base_name)
{
	if (state_type == NO_STATE)
		strcpy(base_name, new_base_name);
	else
		cout << "Unable to set file name while trajectory is being recorded" << endl;
}

void TrajectoryRecorder::setFileName(const char* new_file_name)
{
	if (state_type == NO_STATE)
		strcpy(file_name, new_file_name);
	else
		cout << "Unable to set file name while trajectory is being recorded" << endl;
}

void TrajectoryRecorder::getFileName(char* name)
{
	strcpy(name, file_name);
}

void TrajectoryRecorder::queryFileName()
{
  if (state_type == NO_STATE) {
		cout << "Please enter destination file name for trajectory (without extension): ";
		cin >> file_name;
	} else {
		cout << "Unable to query file name while trajectory is being recorded" << endl;
	}
}

void TrajectoryRecorder::start(StateType type)
{
  if (state_type == NO_STATE) {
		if (type != NO_STATE) {
			state_type = type;
			char *full_path = new char[256];
			if (type == STATE) {
				sprintf(full_path, "%s%s%s", base_name, file_name, ".twd");
				cout << "Writing world trajectory to: " << full_path << endl;
			} else if (type == CONTROL) {
				sprintf(full_path, "%s%s%s", base_name, file_name, ".tcl");
				cout << "Writing control trajectory to: " << full_path << endl;
			}
			
			file.precision(20);
			file.open(full_path);

		} else {
			cout << "NO_STATE is an invalid StateType to start. TrajectoryRecorder was unable to start." << endl;
		}
	} else {
		cout << "Trajectory was already started. Stop it first before starting to record a new trajectory." << endl;
	}
}

void TrajectoryRecorder::stop()
{
  if (state_type != NO_STATE) {
		file << NO_STATE << " ";
		file << "\n";
		file.close();

		char *full_path = new char[256];
		if (state_type == STATE) {
			sprintf(full_path, "%s%s%s", base_name, file_name, ".twd");
		} else if (state_type == CONTROL) {
			sprintf(full_path, "%s%s%s", base_name, file_name, ".tcl");
		}
		cout << "Writing trajectory to " << full_path << " is done" << endl;

		state_type = NO_STATE;

	} else {
		cout << "Trajectory have not been started. Start it first before stopping it." << endl;
	}
}

void TrajectoryRecorder::writeWorldToFile(World* world)
{
  if (state_type == STATE) {
		file << STATE << " ";
		world->writeToFile(file);
	} else if (state_type == CONTROL) {
		cout << "Unable to record world. The TrajectoryRecorder is currently recording a control trajectory." << endl;
	} else {
		cout << "Unable to record world. The TrajectoryRecorder has not started to record." << endl;
	}
}

void TrajectoryRecorder::writeControlToFile(Control* control0, Control* control1)
{
  if (state_type == CONTROL) {
		file << CONTROL << " ";
		control0->writeToFile(file);
		control1->writeToFile(file);
		file << "\n";
	} else if (state_type == STATE) {
		cout << "Unable to record control. The TrajectoryRecorder is currently recording a world trajectory." << endl;
	} else {
		cout << "Unable to record control. The TrajectoryRecorder has not started to record." << endl;
	}
}

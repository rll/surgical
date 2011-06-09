#include "trajectory_recorder.h"



Trajectory_Recorder::Trajectory_Recorder()
{
 strcpy(_fileName_motions, TRAJ_FILE_PATH);
 sprintf(_fileName_threads, "%s.txt", IMAGE_BASE_NAME);
}

Trajectory_Recorder::Trajectory_Recorder(char* fileName_motions_in, char* fileName_threads_in)
{
 strcpy(_fileName_motions, fileName_motions_in);
 strcpy(_fileName_threads, fileName_threads_in);
}



void Trajectory_Recorder::add_motion_to_list(Vector3d& start_pos, Vector3d& end_pos, Vector3d& start_tan, Vector3d& end_tan)
{
  _motions.resize(_motions.size()+1);
  _motions.back().pos_movement = (end_pos-start_pos);
  rotationBetweenTans(start_tan, end_tan, _motions.back().tan_rotation);
}

void Trajectory_Recorder::add_motion_to_list(Vector3d& pos_mov, Matrix3d& tan_rot)
{
  _motions.resize(_motions.size()+1);
  _motions.back().pos_movement = pos_mov;
  _motions.back().tan_rotation = tan_rot;
}

void Trajectory_Recorder::add_motion_to_list(Thread_Motion& motion)
{
  _motions.push_back(motion);
}


void Trajectory_Recorder::add_thread_to_list(Thread* thread)
{
  _threads.push_back(thread);
}


void Trajectory_Recorder::write_motions_to_file()
{
  ofstream trajfile;
  trajfile.precision(10);
  trajfile.open(_fileName_motions);
  for (int i=0; i < _motions.size(); i++)
  {
    trajfile << _motions[i].pos_movement(0) << "   " << _motions[i].pos_movement(1) << "   " <<  _motions[i].pos_movement(2) << "   ";
    trajfile << _motions[i].tan_rotation(0,0) << "   " << _motions[i].tan_rotation(0,1) << "   " << _motions[i].tan_rotation(0,2) << "   ";
    trajfile << _motions[i].tan_rotation(1,0) << "   " << _motions[i].tan_rotation(1,1) << "   " << _motions[i].tan_rotation(1,2) << "   ";
    trajfile << _motions[i].tan_rotation(2,0) << "   " << _motions[i].tan_rotation(2,1) << "   " << _motions[i].tan_rotation(2,2) << "   ";
    trajfile << "\n" << std::endl;
  }
  trajfile.close();

}


void Trajectory_Recorder::write_threads_to_file()
{
  ofstream threadFile;
  threadFile.precision(20);
  threadFile.open(_fileName_threads);
  MatrixXd points(NUM_PTS_TO_SAVE,3);

  if (_threads.size() == 0)
    return;

  //first line consists of length of thread, and pts per thread
  threadFile << _threads[0]->length() << "  " << points.rows() << "\n";

  //write each point for each thread
  for (int i=0; i < _threads.size(); i++)
  {
    Thread* thread = _threads[i];

    thread->getPoints(points);

    for (int i=0; i < points.rows(); i++)
    {
      threadFile << points(i,0) << " " << points(i,1) << " " << points(i,2) << "   ";
    }
   // Vector3d wantedEnd;
   // thread->getWantedEndPosition(wantedEnd);
    //std::cout << "diff last: " << (wantedEnd - points.block(points.rows()-1,0,1,3).transpose()) << std::endl;
    threadFile << "\n";


  }
  threadFile.close();


}






#include "trajectory_recorder.h"



Trajectory_Recorder::Trajectory_Recorder()
{
 strcpy(_fileName_motions, TRAJ_FILE_PATH);
 sprintf(_fileName_threads, "%s.txt", IMAGE_BASE_NAME);
 sprintf(_fileName_thread_points, "%s.points", IMAGE_BASE_NAME);
}

Trajectory_Recorder::Trajectory_Recorder(char* fileName_motions_in, char* fileName_threads_in)
{
 strcpy(_fileName_motions, fileName_motions_in);
 sprintf(_fileName_threads, "%s.txt", fileName_threads_in);
 sprintf(_fileName_thread_points, "%s.points", fileName_threads_in);
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
  MatrixXd points(NUM_PTS_TO_SAVE,3);
  thread->getPoints(points);
  Vector3d positions[2];
  Vector3d tangents[2];

  thread->getWantedEndPosition(positions[1]);
  thread->getWantedEndTangent(tangents[1]);
  thread->getWantedStartPosition(positions[0]);
  thread->getWantedStartTangent(tangents[0]);

  savedThread toSave(points, positions, tangents);
  _threads.push_back(toSave);

  _length = thread->length();
}

void Trajectory_Recorder::add_thread_to_list(MatrixXd& points, Vector3d positions[], Vector3d tangents[])
{
  savedThread toSave(points, positions, tangents);
  _threads.push_back(toSave);
}


void Trajectory_Recorder::setLength(double length)
{
  _length = length;
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

  std::cout << "WRITING TO " << _fileName_threads << std::endl;

  ofstream threadFile;
  threadFile.precision(20);
  threadFile.open(_fileName_threads);
  MatrixXd points(NUM_PTS_TO_SAVE,3);

  if (_threads.size() == 0)
    return;

  threadFile << _length << "  " << NUM_PTS_TO_SAVE << "\n";

  //write each point for each thread
  for (int i=0; i < _threads.size(); i++)
  {
    points = _threads[i].points;

    for (int j=0; j < points.rows(); j++)
    {
      threadFile << points(j,0) << " " << points(j,1) << " " << points(j,2) << "   ";
    }
    Vector3d positions[2];
    Vector3d tangents[2];

    threadFile << _threads[i].positions[0][0] << " " << _threads[i].positions[0][1] << " " << _threads[i].positions[0][2] << "   ";
    threadFile << _threads[i].positions[1][0] << " " << _threads[i].positions[1][1] << " " << _threads[i].positions[1][2] << "   ";
    threadFile << _threads[i].tangents[0][0] << " " << _threads[i].tangents[0][1] << " " << _threads[i].tangents[0][2] << "   ";
    threadFile << _threads[i].tangents[1][0] << " " << _threads[i].tangents[1][1] << " " << _threads[i].tangents[1][2] << "   ";

    
    

   // Vector3d wantedEnd;
   // thread->getWantedEndPosition(wantedEnd);
    //std::cout << "diff last: " << (wantedEnd - points.block(points.rows()-1,0,1,3).transpose()) << std::endl;
    threadFile << "\n";


  }


  /*
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
    Vector3d positions[2];
    Vector3d tangents[2];

    thread->getWantedEndPosition(positions[1]);
    thread->getWantedEndTangent(tangents[1]);
    thread->getWantedStartPosition(positions[0]);
    thread->getWantedStartTangent(tangents[0]);

    threadFile << positions[0][0] << " " << positions[0][1] << " " << positions[0][2] << "   ";
    threadFile << positions[1][0] << " " << positions[1][1] << " " << positions[1][2] << "   ";
    threadFile << tangents[0][0] << " " << tangents[0][1] << " " << tangents[0][2] << "   ";
    threadFile << tangents[1][0] << " " << tangents[1][1] << " " << tangents[1][2] << "   ";




   // Vector3d wantedEnd;
   // thread->getWantedEndPosition(wantedEnd);
    //std::cout << "diff last: " << (wantedEnd - points.block(points.rows()-1,0,1,3).transpose()) << std::endl;
    threadFile << "\n";


  }
  */
  threadFile.close();
  remove(_fileName_thread_points);


}






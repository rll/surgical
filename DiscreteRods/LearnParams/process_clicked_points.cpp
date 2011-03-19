#include "../trajectory_recorder.h"
#include "../thread_discrete.h"

#include <iostream>
#include <fstream>

#define INFILE "../../vision/captures/suturepurple_points.txt"
#define OUTFILE "suturepurple_processed"
#define PROJECT false
#define NUM_PTS 34
#define NUM_CAMS 3

int main (int argc, char * argv[])
{
  Trajectory_Recorder traj_recorder(OUTFILE);


  ifstream points_in;
  points_in.open(INFILE);

  vector<Vector3d> points(NUM_PTS);
  vector<Vector3d> points_reversed(NUM_PTS);
  vector<double> twist_angles(NUM_PTS);
  Matrix3d start_rot;
  Matrix3d end_rot;
  double last_holonomy = 0;
  double num_rotation = 0.0;

  while (!points_in.eof())
  {
    double set_num;
    points_in >> set_num;

    //seems to read one last time when data ends...this fixes it
    if (points_in.eof())
      break;

    double total_angle = -( (double) (( (int)set_num - 1)%10))* 30.0/360.0 * 2.0 * M_PI;

    for (int i=0; i < NUM_PTS; i++)
    {
      points_in >> points[i](0) >> points[i](1) >> points[i](2);
    }
    
    //skip data from click locations
    double for_skip;
    for (int i=0; i < NUM_PTS*NUM_CAMS*2; i++)
    {
      points_in >> for_skip;
    }

    //reverse order of points
    for (int i=0; i < NUM_PTS; i++)
    {
      points_reversed[i] = points[NUM_PTS-1-i];
    }

    //find angle from holonomy
    for (int i=0; i < NUM_PTS; i++)
    {
      twist_angles[i] = 0.0;
    }


    Vector3d edge1 = points_reversed[1] - points_reversed[0];
    start_rot.col(0) = edge1.normalized();
		start_rot.col(1) = -(Vector3d::UnitY() - (Vector3d::UnitY().dot(edge1.normalized()))*edge1.normalized()).normalized();
    start_rot.col(2) = start_rot.col(0).cross(start_rot.col(1)).normalized();





    if ( ((int)set_num - 1)%10 == 0)
    {
      last_holonomy = 0.0;
      num_rotation = 0.0;
      std::cout << std::endl;
    }

    Thread with_no_rot(points_reversed, twist_angles, start_rot);
    if (PROJECT)
      with_no_rot.project_length_constraint_slow();
    double holonomy = with_no_rot.calculate_holonomy();    

    double holonomy_to_use;
    if (abs(holonomy - last_holonomy) > M_PI)
    {
      num_rotation += (angle_diff(holonomy, last_holonomy) ? -1.0 : 1.0);
    }
    holonomy_to_use = holonomy + num_rotation*2*M_PI;
    last_holonomy = holonomy;
   



    double angle_for_pieces = total_angle - holonomy_to_use;
    double angle_per_piece = angle_for_pieces/((double)(NUM_PTS-2));
    for (int i=0; i < NUM_PTS-1; i++)
    {
      twist_angles[i] = ((double)i)*angle_per_piece;
    }
    twist_angles.back() = twist_angles[twist_angles.size()-2];

    Thread to_save(points_reversed, twist_angles, start_rot);
    if (PROJECT)
      to_save.project_length_constraint_slow();
    traj_recorder.add_thread_to_list(to_save);

    std::cout << "holonomy: " << holonomy_to_use <<  " \ttotal thread: " << angle_for_pieces << std::endl;

  }

  traj_recorder.write_threads_to_file();


}


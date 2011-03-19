
#include "../trajectory_recorder.h"
#include "../thread_discrete.h"

#include <iostream>
#include <fstream>

#define INFILE "../../vision/captures/ribbon_dots_points.txt"
#define OUTFILE "ribbon_dots_processed_projected"
#define PROJECT true
#define NUM_PTS 34
#define NUM_CAMS 3



int main (int argc, char * argv[])
{
  Trajectory_Recorder traj_recorder(OUTFILE);


  ifstream points_in;
  points_in.open(INFILE);

  vector<Vector3d> points(NUM_PTS*2);
	vector<Vector3d> center_points(NUM_PTS);
	vector<Vector3d> dir_up(NUM_PTS);
  vector<Vector3d> center_points_reversed(NUM_PTS);
  vector<Vector3d> dir_up_reversed(NUM_PTS);
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

    double total_angle = ( (double) (( (int)set_num - 1)%10))* 30.0/360.0 * 2.0 * M_PI;

    for (int i=0; i < NUM_PTS*2; i++)
    {
      points_in >> points[i](0) >> points[i](1) >> points[i](2);
    }
    
    //skip data from click locations
    double for_skip;
    for (int i=0; i < NUM_PTS*NUM_CAMS*2*2; i++)
    {
      points_in >> for_skip;
    }

    //find center of points
    for (int i=0; i < NUM_PTS; i++)
    {
        center_points[i] = (points[2*i] - points[2*i+1])/2.0 + points[2*i+1];
    }

    //point up to find frame
    for (int i=0; i < NUM_PTS; i++)
    {
        dir_up[i] = (points[2*i]-center_points[i]).normalized();
    }


    //reverse order of points
    for (int i=0; i < NUM_PTS; i++)
    {
      center_points_reversed[i] = center_points[NUM_PTS-1-i];
			dir_up_reversed[i] = dir_up[NUM_PTS-1-i];
    }


    //first need bishop frame
    for (int i=0; i < NUM_PTS; i++)
    {
      twist_angles[i] = 0.0;
    }
    Vector3d edge1 = center_points_reversed[1] - center_points_reversed[0];
    //Vector3d edge2 = center_points_reversed[2] - center_points_reversed[1];
    start_rot.col(0) = edge1.normalized();
	start_rot.col(1) = -(dir_up_reversed[0] - (dir_up_reversed[0].dot(edge1.normalized()))*edge1.normalized()).normalized();
    start_rot.col(2) = start_rot.col(0).cross(start_rot.col(1)).normalized();

		//std::cout << "dot init rot: " << start_rot.col(0).dot(start_rot.col(1)) << " " << start_rot.col(0).dot(start_rot.col(2)) << " " << start_rot.col(1).dot(start_rot.col(2)) << std::endl;


    if ( ((int)set_num - 1)%10 == 0)
    {
      last_holonomy = 0.0;
      num_rotation = 0.0;
      std::cout << std::endl;
    }


    Thread with_no_rot(center_points_reversed, twist_angles, start_rot);

    if (PROJECT) 
    {
        with_no_rot.project_length_constraint();
    }

    vector<Vector3d> with_no_rot_points;
    vector<Matrix3d> with_no_rot_material_frames;
    with_no_rot.get_thread_data(with_no_rot_points, with_no_rot_material_frames);

    double holonomy_to_use;
    double holonomy = with_no_rot.calculate_holonomy();    
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
      //twist_angles[i] = ((double)i)*angle_per_piece;
      twist_angles[i] = 0;
    }
    twist_angles.back() = twist_angles[twist_angles.size()-2];
		for (int i=1; i < NUM_PTS-1; i++)
		{
			Matrix3d material_frame_next;
			Vector3d edge = center_points_reversed[i+1]-center_points_reversed[i];
			material_frame_next.col(0) = edge.normalized();
			material_frame_next.col(1) = -(dir_up_reversed[i] - (dir_up_reversed[i].dot(edge.normalized()))*edge.normalized()).normalized();
			material_frame_next.col(2) = material_frame_next.col(0).cross(material_frame_next.col(1));

			Matrix3d init_with_last_twist;
		    /*
            init_with_last_twist = Eigen::AngleAxisd(twist_angles[i-1], with_no_rot_material_frames[i].col(0).normalized())*with_no_rot_material_frames[i];
			
			//double angle_err_frames = angle_mismatch(init_with_last_twist, material_frame_next);
			double angle_err_frames = angle_mismatch(material_frame_next, init_with_last_twist);
			double angle_add = atan2( sin(angle_err_frames - twist_angles[i-1]), cos(angle_err_frames - twist_angles[i-1]));
			twist_angles[i] = twist_angles[i-1] + angle_add;
			*/
            
            init_with_last_twist = Eigen::AngleAxisd(twist_angles[i], with_no_rot_material_frames[i].col(0).normalized())*with_no_rot_material_frames[i];
			
			//double angle_err_frames = angle_mismatch(init_with_last_twist, material_frame_next);
			double angle_err_frames = angle_mismatch(material_frame_next, init_with_last_twist);
			double angle_add = atan2( sin(angle_err_frames - twist_angles[i]), cos(angle_err_frames - twist_angles[i]));
			twist_angles[i] = twist_angles[i] + angle_add;
			
            
		}

        for (int i = 1; i < NUM_PTS-1; i++) { 
            if (twist_angles[i] - twist_angles[i-1] > M_PI) {
                twist_angles[i] = twist_angles[i] - 2 * M_PI;
            } else if (twist_angles[i-1] - twist_angles[i] > M_PI) {
                twist_angles[i] = twist_angles[i] + 2 * M_PI;
            }
            std::cout << twist_angles[i] << " ";
            
        }
		std::cout << std::endl;
		//
		
		Thread to_save(center_points_reversed, twist_angles, start_rot);
		if (PROJECT)
			to_save.project_length_constraint();
    traj_recorder.add_thread_to_list(to_save);
	
 
		/*
    double holonomy = with_no_rot.calculate_holonomy();    

    double holonomy_to_use;
    if (abs(holonomy - last_holonomy) > M_PI)
    {
      num_rotation += 1.0;
    }
    holonomy_to_use = holonomy + num_rotation*2*M_PI;
    last_holonomy = holonomy;


    double angle_for_pieces = total_angle - holonomy_to_use;
		std::cout << "angle for pieces: " << angle_for_pieces << std::endl;
		*/
  }

  traj_recorder.write_threads_to_file();


}


#include "planner_lib.h"
#include "global_filenames.h"
#include "../DiscreteRods/trajectory_reader.h"
#include "../DiscreteRods/trajectory_recorder.h"

void deleteAllThreads(vector<Thread*> toDelete);
void wrap_controls_extra_vector(vector<VectorXd>& controls, vector<vector<VectorXd> >& controls_wrapped);


int main(int argc, char* argv[]) {
  
 if (argc < 3)
 {
    std::cerr << "please provide arguments: start_ind    end_ind   dimension" << std::endl;
 }

 srand(0);
 srand48(0);

  int trajs_start_ind = atoi (argv[1]);
  int trajs_end_ind = atoi (argv[2]);
  int num_links = atoi (argv[3]);
  vector<Thread> start_threads;
  vector<Thread> goal_threads;


  char start_threads_filename[256];
  char goal_threads_filename[256];
  sprintf(start_threads_filename, "%s/%s_%d", BASEFOLDER_INITDATA, BASENAME_STARTTHREADS, num_links);
  sprintf(goal_threads_filename, "%s/%s_%d", BASEFOLDER_INITDATA, BASENAME_GOALTHREADS, num_links);

  Trajectory_Reader start_threads_reader(start_threads_filename);
  Trajectory_Reader goal_threads_reader(goal_threads_filename);
  start_threads_reader.read_threads_from_file();
  goal_threads_reader.read_threads_from_file();
  start_threads = start_threads_reader.get_all_threads();
  goal_threads = goal_threads_reader.get_all_threads();
  
  

  
  for (int thread_ind = trajs_start_ind; thread_ind <= std::min(trajs_end_ind, (int)(start_threads.size()-1)); thread_ind++)
  {
    //for now, dont call interpolate ends
    //after running rrt, also call linearizetogoal

    char linearize_only_filename[256];
    char interpolate_end_and_linear_filename[256];
    char interpolate_point_and_linearize_filename[256];
    char sqp_openloop_filename[256];
    char sqp_closedloop_filename[256];
    char RRT_filename[256];
    char RRT_SQP_openloop_filename[256];
    char RRT_SQP_closedloop_filename[256];
    char RRT_SQP_closedloop_onlylast_filename[256];

    
    sprintf(linearize_only_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_LINEARIZE_ONLY, thread_ind);
    //sprintf(interpolate_end_and_linear_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_INTERPOLATE_ENDS_AND_LINEARIZE, thread_ind);
    sprintf(interpolate_point_and_linearize_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_INTERPOLATE_POINT_AND_LINEARIZE, thread_ind);
    sprintf(sqp_openloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_SQP_OPENLOOP, thread_ind);
    sprintf(sqp_closedloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_SQP_CLOSEDLOOP, thread_ind);
    sprintf(RRT_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT, thread_ind);
    sprintf(RRT_SQP_openloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_SQP_OPENLOOP, thread_ind);
    sprintf(RRT_SQP_closedloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_SQP_CLOSEDLOOP, thread_ind);
    sprintf(RRT_SQP_closedloop_onlylast_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_SQP_CLOSEDLOOP_ONLYLAST, thread_ind);


    Trajectory_Recorder linearize_only_recorder(linearize_only_filename);
    //Trajectory_Recorder interpolate_end_and_linear_recorder(interpolate_end_and_linear_filename);
    Trajectory_Recorder interpolate_point_and_linearize_recorder(interpolate_point_and_linearize_filename);
    Trajectory_Recorder sqp_openloop_recorder(sqp_openloop_filename);
    Trajectory_Recorder sqp_closedloop_recorder(sqp_closedloop_filename);
    Trajectory_Recorder RRT_recorder(RRT_filename);
    Trajectory_Recorder RRT_SQP_openloop_recorder(RRT_SQP_openloop_filename);
    Trajectory_Recorder RRT_SQP_closedloop_recorder(RRT_SQP_closedloop_filename);
    Trajectory_Recorder RRT_SQP_closedloop_onlylast_recorder(RRT_SQP_closedloop_onlylast_filename);


    vector<Thread*> current_traj;
    vector<Thread*> interpolated_points;
    vector<Thread*> interpolated_ends;

    interpolateEndsTrajectory(&start_threads[thread_ind], &goal_threads[thread_ind], interpolated_ends);
    interpolatePointsTrajectory(&start_threads[thread_ind], &goal_threads[thread_ind], interpolated_points);
  /*
    //linearize only
    linearizeToGoal(&start_threads[thread_ind], &goal_threads[thread_ind], current_traj);
    linearize_only_recorder.add_threads_to_list(current_traj);
    deleteAllThreads(current_traj);

    
    //interpolate ends and follow
    //linearizeViaTrajectory(interpolated_ends, current_traj);
    //interpolate_end_and_linear_recorder.add_threads_to_list(current_traj);
    //deleteAllThreads(current_traj);
    

    //interpolate points and follow
    linearizeViaTrajectory(interpolated_points, current_traj);
    interpolate_point_and_linearize_recorder.add_threads_to_list(current_traj);
    deleteAllThreads(current_traj);
*/

    //generate SQP data
    vector<Thread*> sqp_points;
    vector<VectorXd> sqp_controls;
    vector<vector<VectorXd> > sqp_controls_wrapped;
    solveSQP(interpolated_points, sqp_points, sqp_controls);
    wrap_controls_extra_vector(sqp_controls, sqp_controls_wrapped);
  
    //sqp open loop
    openLoopController(sqp_points, sqp_controls, current_traj);
    sqp_openloop_recorder.add_threads_to_list(current_traj);
    deleteAllThreads(current_traj);

    //sqp closed loop
    closedLoopLinearizationController(sqp_points, sqp_controls_wrapped, current_traj);
    sqp_closedloop_recorder.add_threads_to_list(current_traj);
    deleteAllThreads(current_traj);

    
    //generate RRT
    vector<Thread*> RRT_traj;
    vector<vector<VectorXd> > RRT_controls;
    RRTPlanner(&start_threads[thread_ind], &goal_threads[thread_ind], 0, RRT_traj, RRT_controls);
    RRT_recorder.add_threads_to_list(RRT_traj);


    //smooth with SQP, openloop
    vector<Thread*> rrt_sqp_traj;
    vector<VectorXd> rrt_sqp_controls;
    vector<vector<VectorXd> > rrt_sqp_controls_wrapped;
    solveSQP(RRT_traj, rrt_sqp_traj, rrt_sqp_controls);
    wrap_controls_extra_vector(rrt_sqp_controls, rrt_sqp_controls_wrapped);


    //playback sqp openloop
    openLoopController(rrt_sqp_traj, rrt_sqp_controls, current_traj);
    RRT_SQP_openloop_recorder.add_threads_to_list(current_traj);
    deleteAllThreads(current_traj);

    //playback sqp closedloop
    closedLoopLinearizationController(rrt_sqp_traj, rrt_sqp_controls_wrapped, current_traj);
    RRT_SQP_closedloop_recorder.add_threads_to_list(current_traj);
    deleteAllThreads(current_traj);


    //solve sqp from end of rrt
    vector<Thread*> rrt_endtogoal_interpolate;
    vector<Thread*> rrt_endtogoal_sqp_traj;
    vector<VectorXd> rrt_endtogoal_sqp_controls;
    vector<vector<VectorXd> > rrt_endtogoal_sqp_controls_wrapped;
    interpolatePointsTrajectory(RRT_traj.back(), &goal_threads[thread_ind], rrt_endtogoal_interpolate);
    solveSQP(rrt_endtogoal_interpolate, rrt_endtogoal_sqp_traj, rrt_endtogoal_sqp_controls);
    wrap_controls_extra_vector(rrt_endtogoal_sqp_controls, rrt_endtogoal_sqp_controls_wrapped);
    closedLoopLinearizationController(rrt_endtogoal_sqp_traj, rrt_endtogoal_sqp_controls_wrapped, current_traj);
    RRT_SQP_closedloop_onlylast_recorder.add_threads_to_list(current_traj);
    deleteAllThreads(current_traj);




    
    //delete unneeded?
    //deleteAllThreads(interpolated_ends);
    deleteAllThreads(interpolated_points);
    deleteAllThreads(sqp_points);
    deleteAllThreads(rrt_sqp_traj);
    deleteAllThreads(rrt_endtogoal_interpolate);
    deleteAllThreads(rrt_endtogoal_sqp_traj);


    linearize_only_recorder.write_threads_to_file();
    //interpolate_end_and_linear_recorder.write_threads_to_file();
    interpolate_point_and_linearize_recorder.write_threads_to_file();
    sqp_openloop_recorder.write_threads_to_file();
    sqp_closedloop_recorder.write_threads_to_file();
    RRT_recorder.write_threads_to_file();
    RRT_SQP_openloop_recorder.write_threads_to_file();
    RRT_SQP_closedloop_recorder.write_threads_to_file();
    RRT_SQP_closedloop_onlylast_recorder.write_threads_to_file();



  } 


}


void deleteAllThreads(vector<Thread*> toDelete)
{
  for (int i=0; i < toDelete.size(); i++)
  {
    delete toDelete[i];
  }
  toDelete.resize(0);
}


void wrap_controls_extra_vector(vector<VectorXd>& controls, vector<vector<VectorXd> >& controls_wrapped)
{
  for (int i=0; i < controls.size(); i++)
  {
    vector<VectorXd> this_control_wrapper(1);
    this_control_wrapper[0] = controls[i];
    controls_wrapped.push_back(this_control_wrapper);
  }
}






  






  






  



#include "planner_lib.h"
#include "global_filenames.h"
#include "../DiscreteRods/trajectory_reader.h"
#include "../DiscreteRods/trajectory_recorder.h"
#include <ostream>
#include <boost/timer.hpp>

void deleteAllThreads(vector<Thread*>& toDelete);
void wrap_controls_extra_vector(vector<VectorXd>& controls, vector<vector<VectorXd> >& controls_wrapped);
void Copy_Threads(const vector<Thread*>& to_copy, vector<Thread*>& copy);

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
  vector<Thread*> start_threads;
  vector<Thread*> goal_threads;


  char start_threads_filename[256];
  char goal_threads_filename[256];
  sprintf(start_threads_filename, "%s/%s_%d", BASEFOLDER_INITDATA, BASENAME_STARTTHREADS, num_links);
  sprintf(goal_threads_filename, "%s/%s_%d", BASEFOLDER_INITDATA, BASENAME_GOALTHREADS, num_links);

  Trajectory_Reader start_threads_reader(start_threads_filename);
  Trajectory_Reader goal_threads_reader(goal_threads_filename);
  start_threads_reader.read_threads_from_file();
  goal_threads_reader.read_threads_from_file();
  
  //start_threads = start_threads_reader.get_all_threads();
  //goal_threads = goal_threads_reader.get_all_threads();
  

  start_threads_reader.get_all_threads(start_threads);
  goal_threads_reader.get_all_threads(goal_threads);
  

  
  for (int thread_ind = trajs_start_ind; thread_ind <= std::min(trajs_end_ind, (int)(start_threads.size()-1)); thread_ind++)
  {
    //for now, dont call interpolate ends
    //after running rrt, also call linearizetogoal

    char sqp_namestring[256];
    sprintf(sqp_namestring, "%d_%d", num_links, thread_ind);


    char results_filename[256]; 
    char linearize_only_filename[256];
    char interpolate_end_and_linear_filename[256];
    char interpolate_point_and_linearize_filename[256];
    char sqp_openloop_filename[256];
    char sqp_closedloop_filename[256];
    char RRT_filename[256];
    char RRT_SQP_openloop_filename[256];
    char RRT_SQP_closedloop_filename[256];
    char RRT_SQP_closedloop_onlylast_filename[256];
    char RRT_dim1_filename[256];
    char RRT_dim1_SQP_openloop_filename[256];
    char RRT_dim1_SQP_closedloop_filename[256];
    char RRT_dim1_SQP_closedloop_onlylast_filename[256];
    char RRT_dim2_filename[256];
    char RRT_dim2_SQP_openloop_filename[256];
    char RRT_dim2_SQP_closedloop_filename[256];
    char RRT_dim2_SQP_closedloop_onlylast_filename[256];
  
  
  
    sprintf(results_filename, "%s/%d_%d_data.txt", BASEFOLDER_RESULTS, num_links, thread_ind); 
    sprintf(linearize_only_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_LINEARIZE_ONLY, thread_ind);
    //sprintf(interpolate_end_and_linear_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_INTERPOLATE_ENDS_AND_LINEARIZE, thread_ind);
    sprintf(interpolate_point_and_linearize_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_INTERPOLATE_POINT_AND_LINEARIZE, thread_ind);
    sprintf(sqp_openloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_SQP_OPENLOOP, thread_ind);
    sprintf(sqp_closedloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_SQP_CLOSEDLOOP, thread_ind);
    sprintf(RRT_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT, thread_ind);
    sprintf(RRT_SQP_openloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_SQP_OPENLOOP, thread_ind);
    sprintf(RRT_SQP_closedloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_SQP_CLOSEDLOOP, thread_ind);
    sprintf(RRT_SQP_closedloop_onlylast_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_SQP_CLOSEDLOOP_ONLYLAST, thread_ind);
    
    sprintf(RRT_dim1_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim1, thread_ind);
    sprintf(RRT_dim1_SQP_openloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim1_SQP_OPENLOOP, thread_ind);
    sprintf(RRT_dim1_SQP_closedloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim1_SQP_CLOSEDLOOP, thread_ind);
    sprintf(RRT_dim1_SQP_closedloop_onlylast_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim1_SQP_CLOSEDLOOP_ONLYLAST, thread_ind);

    sprintf(RRT_dim2_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim2, thread_ind);
    sprintf(RRT_dim2_SQP_openloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim2_SQP_OPENLOOP, thread_ind);
    sprintf(RRT_dim2_SQP_closedloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim2_SQP_CLOSEDLOOP, thread_ind);
    sprintf(RRT_dim2_SQP_closedloop_onlylast_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim2_SQP_CLOSEDLOOP_ONLYLAST, thread_ind);


    Timer timer; 
    ofstream results_file;
    results_file.open(results_filename);


    Trajectory_Recorder linearize_only_recorder(linearize_only_filename);
    //Trajectory_Recorder interpolate_end_and_linear_recorder(interpolate_end_and_linear_filename);
    Trajectory_Recorder interpolate_point_and_linearize_recorder(interpolate_point_and_linearize_filename);
    Trajectory_Recorder sqp_openloop_recorder(sqp_openloop_filename);
    Trajectory_Recorder sqp_closedloop_recorder(sqp_closedloop_filename);
    Trajectory_Recorder RRT_recorder(RRT_filename);
    Trajectory_Recorder RRT_SQP_openloop_recorder(RRT_SQP_openloop_filename);
    Trajectory_Recorder RRT_SQP_closedloop_recorder(RRT_SQP_closedloop_filename);
    Trajectory_Recorder RRT_SQP_closedloop_onlylast_recorder(RRT_SQP_closedloop_onlylast_filename);
    
    Trajectory_Recorder RRT_dim1_recorder(RRT_dim1_filename);
    Trajectory_Recorder RRT_dim1_SQP_openloop_recorder(RRT_dim1_SQP_openloop_filename);
    Trajectory_Recorder RRT_dim1_SQP_closedloop_recorder(RRT_dim1_SQP_closedloop_filename);
    Trajectory_Recorder RRT_dim1_SQP_closedloop_onlylast_recorder(RRT_dim1_SQP_closedloop_onlylast_filename);
    Trajectory_Recorder RRT_dim2_recorder(RRT_dim2_filename);
    Trajectory_Recorder RRT_dim2_SQP_openloop_recorder(RRT_dim2_SQP_openloop_filename);
    Trajectory_Recorder RRT_dim2_SQP_closedloop_recorder(RRT_dim2_SQP_closedloop_filename);
    Trajectory_Recorder RRT_dim2_SQP_closedloop_onlylast_recorder(RRT_dim2_SQP_closedloop_onlylast_filename);




    vector<Thread*> linearize_only_traj(0);
    //vector<Thread*> interpolate_end_and_linear_traj;
    vector<Thread*> interpolate_point_and_linearize_traj(0);
    vector<Thread*> sqp_openloop_traj(0);
    vector<Thread*> sqp_closedloop_traj(0);
    vector<Thread*> RRT_traj(0);
    vector<Thread*> RRT_SQP_openloop_traj(0);
    vector<Thread*> RRT_SQP_closedloop_traj(0);
    vector<Thread*> RRT_SQP_closedloop_onlylast_traj(0);

    vector<Thread*> RRT_dim1_traj(0);
    vector<Thread*> RRT_dim1_SQP_openloop_traj(0);
    vector<Thread*> RRT_dim1_SQP_closedloop_traj(0);
    vector<Thread*> RRT_dim1_SQP_closedloop_onlylast_traj(0);

    vector<Thread*> RRT_dim2_traj(0);
    vector<Thread*> RRT_dim2_SQP_openloop_traj(0);
    vector<Thread*> RRT_dim2_SQP_closedloop_traj(0);
    vector<Thread*> RRT_dim2_SQP_closedloop_onlylast_traj(0);

    vector<Thread*> interpolated_points;
    vector<Thread*> interpolated_ends;

    //interpolateEndsTrajectory(start_threads[thread_ind], goal_threads[thread_ind], interpolated_ends);
    interpolatePointsTrajectory(start_threads[thread_ind], goal_threads[thread_ind], interpolated_points);
    
    timer.restart();
    //linearize only
    linearizeToGoal(start_threads[thread_ind], goal_threads[thread_ind], linearize_only_traj);
    linearize_only_recorder.add_threads_to_list(linearize_only_traj);
    results_file << "LINEARIZE_ONLY," 
      << cost_metric(linearize_only_traj.back(), goal_threads[thread_ind])
      << ","
      << timer.elapsed() 
      << endl; 
//    deleteAllThreads(current_traj);

    
    //interpolate ends and follow
    //linearizeViaTrajectory(interpolated_ends, interpolate_end_and_linearize_traj);
    //interpolate_end_and_linear_recorder.add_threads_to_list(interpolate_end_and_linearize_traj);
    //deleteAllThreads(current_traj);

    timer.restart();
    //interpolate points and follow
    linearizeViaTrajectory(interpolated_points, interpolate_point_and_linearize_traj);
    interpolate_point_and_linearize_recorder.add_threads_to_list(interpolate_point_and_linearize_traj);
 //   deleteAllThreads(current_traj);
    results_file << "LINEARIZE_VIA_TRAJECTORY," 
      << cost_metric(interpolate_point_and_linearize_traj.back(), goal_threads[thread_ind])
      << ","
      << timer.elapsed() 
      << endl; 

    timer.restart();
    //generate SQP data
    vector<Thread*> sqp_points;
    vector<VectorXd> sqp_controls;
    vector<vector<VectorXd> > sqp_controls_wrapped;
    solveSQP(interpolated_points, sqp_points, sqp_controls, sqp_namestring);
    double SQP_SOLVER_TIME = timer.elapsed();
    wrap_controls_extra_vector(sqp_controls, sqp_controls_wrapped);

    timer.restart();
    //sqp open loop
    openLoopController(sqp_points, sqp_controls, sqp_openloop_traj);
    sqp_openloop_recorder.add_threads_to_list(sqp_openloop_traj);
    results_file << "OPEN_LOOP_SQP," 
      << cost_metric(sqp_openloop_traj.back(), goal_threads[thread_ind])
      << ","
      << timer.elapsed() + SQP_SOLVER_TIME 
      << endl; 

    //deleteAllThreads(current_traj);
      
    timer.restart();
    //sqp closed loop
    closedLoopLinearizationController(sqp_points, sqp_controls_wrapped, sqp_closedloop_traj);
    sqp_closedloop_recorder.add_threads_to_list(sqp_closedloop_traj);

    results_file << "CLOSED_LOOP_SQP," 
      << cost_metric(sqp_closedloop_traj.back(), goal_threads[thread_ind])
      << ","
      << timer.elapsed() + SQP_SOLVER_TIME 
      << endl; 

    //deleteAllThreads(current_traj);

    
    timer.restart(); 
    //generate RRT
    vector<vector<VectorXd> > RRT_controls;
    RRTPlanner(start_threads[thread_ind], goal_threads[thread_ind], 0, RRT_traj, RRT_controls);
    RRT_recorder.add_threads_to_list(RRT_traj);
    double RRT_PLANNER_TIME = timer.elapsed(); 
    results_file << "RRT_PLANNER," 
      << cost_metric(RRT_traj.back(), goal_threads[thread_ind])
      << ","
      << RRT_PLANNER_TIME
      << endl;

    
    //add goal to RRT for sqp
    vector<Thread*> RRT_traj_togoal;
    traj_subsampling(RRT_traj, RRT_traj_togoal);
    RRT_traj_togoal.push_back(new Thread(*goal_threads[thread_ind]));
  
    timer.restart();
    //smooth with SQP, openloop
    vector<Thread*> rrt_sqp_traj;
    vector<VectorXd> rrt_sqp_controls;
    vector<vector<VectorXd> > rrt_sqp_controls_wrapped;
    solveSQP(RRT_traj_togoal, rrt_sqp_traj, rrt_sqp_controls, sqp_namestring);
    wrap_controls_extra_vector(rrt_sqp_controls, rrt_sqp_controls_wrapped);
    double RRT_SQP_SMOOTHING_TIME = timer.elapsed(); 

    timer.restart();
    //playback sqp openloop
    openLoopController(rrt_sqp_traj, rrt_sqp_controls, RRT_SQP_openloop_traj);
    RRT_SQP_openloop_recorder.add_threads_to_list(RRT_SQP_openloop_traj);

    results_file << "RRT_PLANNER_SQP_OPENLOOP_SMOOTHER," 
      << cost_metric(RRT_SQP_openloop_traj.back(), goal_threads[thread_ind])
      << ","
      << RRT_PLANNER_TIME + RRT_SQP_SMOOTHING_TIME + timer.elapsed()
      << endl;
    //deleteAllThreads(current_traj);

    timer.restart(); 
    //playback sqp closedloop
    closedLoopLinearizationController(rrt_sqp_traj, rrt_sqp_controls_wrapped, RRT_SQP_closedloop_traj);
    RRT_SQP_closedloop_recorder.add_threads_to_list(RRT_SQP_closedloop_traj);
    //deleteAllThreads(current_traj);

    results_file << "RRT_PLANNER_SQP_CLOSEDLOOP_SMOOTHER," 
      << cost_metric(RRT_SQP_closedloop_traj.back(), goal_threads[thread_ind])
      << ","
      << RRT_PLANNER_TIME + RRT_SQP_SMOOTHING_TIME + timer.elapsed()
      << endl;

    timer.restart(); 
    //solve sqp from end of rrt
    vector<Thread*> rrt_endtogoal_interpolate;
    vector<Thread*> rrt_endtogoal_sqp_traj;
    vector<VectorXd> rrt_endtogoal_sqp_controls;
    vector<vector<VectorXd> > rrt_endtogoal_sqp_controls_wrapped;
    interpolatePointsTrajectory(RRT_traj.back(), goal_threads[thread_ind], rrt_endtogoal_interpolate);
    solveSQP(rrt_endtogoal_interpolate, rrt_endtogoal_sqp_traj, rrt_endtogoal_sqp_controls, sqp_namestring);
    wrap_controls_extra_vector(rrt_endtogoal_sqp_controls, rrt_endtogoal_sqp_controls_wrapped);
    closedLoopLinearizationController(rrt_endtogoal_sqp_traj, rrt_endtogoal_sqp_controls_wrapped, RRT_SQP_closedloop_onlylast_traj);
    RRT_SQP_closedloop_onlylast_recorder.add_threads_to_list(RRT_traj);
    RRT_SQP_closedloop_onlylast_recorder.add_threads_to_list(RRT_SQP_closedloop_onlylast_traj);
    //deleteAllThreads(current_traj);

    results_file << "RRT_PLANNER_SQP_CLOSEDLOOP_ONLY_LAST," 
      << cost_metric(RRT_SQP_closedloop_onlylast_traj.back(), goal_threads[thread_ind])
      << ","
      << RRT_PLANNER_TIME + timer.elapsed()
      << endl;


/***********************************************
 * DIM REDUCE 1
***********************************************/ 

    timer.restart();
    //generate RRT
    vector<vector<VectorXd> > RRT_dim1_controls;
    RRTPlanner(start_threads[thread_ind], goal_threads[thread_ind], 1, RRT_dim1_traj, RRT_dim1_controls);
    RRT_dim1_recorder.add_threads_to_list(RRT_dim1_traj);
    double RRT_dim1_PLANNER_TIME = timer.elapsed(); 
    results_file << "RRT_dim1_PLANNER," 
      << cost_metric(RRT_dim1_traj.back(), goal_threads[thread_ind])
      << ","
      << RRT_dim1_PLANNER_TIME
      << endl;

    
    //add goal to RRT_dim1 for sqp
    vector<Thread*> RRT_dim1_traj_togoal;
    traj_subsampling(RRT_dim1_traj, RRT_dim1_traj_togoal);
    RRT_dim1_traj_togoal.push_back(new Thread(*goal_threads[thread_ind]));
  
    timer.restart();
    //smooth with SQP, openloop
    vector<Thread*> rrt_dim1_sqp_traj;
    vector<VectorXd> rrt_dim1_sqp_controls;
    vector<vector<VectorXd> > rrt_dim1_sqp_controls_wrapped;
    solveSQP(RRT_dim1_traj_togoal, rrt_dim1_sqp_traj, rrt_dim1_sqp_controls, sqp_namestring);
    wrap_controls_extra_vector(rrt_dim1_sqp_controls, rrt_dim1_sqp_controls_wrapped);
    double RRT_dim1_SQP_SMOOTHING_TIME = timer.elapsed(); 

    timer.restart();
    //playback sqp openloop
    openLoopController(rrt_dim1_sqp_traj, rrt_dim1_sqp_controls, RRT_dim1_SQP_openloop_traj);
    RRT_dim1_SQP_openloop_recorder.add_threads_to_list(RRT_dim1_SQP_openloop_traj);

    results_file << "RRT_dim1_PLANNER_SQP_OPENLOOP_SMOOTHER," 
      << cost_metric(RRT_dim1_SQP_openloop_traj.back(), goal_threads[thread_ind])
      << ","
      << RRT_dim1_PLANNER_TIME + RRT_dim1_SQP_SMOOTHING_TIME + timer.elapsed()
      << endl;
    //deleteAllThreads(current_traj);

    timer.restart(); 
    //playback sqp closedloop
    closedLoopLinearizationController(rrt_dim1_sqp_traj, rrt_dim1_sqp_controls_wrapped, RRT_dim1_SQP_closedloop_traj);
    RRT_dim1_SQP_closedloop_recorder.add_threads_to_list(RRT_dim1_SQP_closedloop_traj);
    //deleteAllThreads(current_traj);

    results_file << "RRT_dim1_PLANNER_SQP_CLOSEDLOOP_SMOOTHER," 
      << cost_metric(RRT_dim1_SQP_closedloop_traj.back(), goal_threads[thread_ind])
      << ","
      << RRT_dim1_PLANNER_TIME + RRT_dim1_SQP_SMOOTHING_TIME + timer.elapsed()
      << endl;

    timer.restart(); 
    //solve sqp from end of rrt_dim1
    vector<Thread*> rrt_dim1_endtogoal_interpolate;
    vector<Thread*> rrt_dim1_endtogoal_sqp_traj;
    vector<VectorXd> rrt_dim1_endtogoal_sqp_controls;
    vector<vector<VectorXd> > rrt_dim1_endtogoal_sqp_controls_wrapped;
    interpolatePointsTrajectory(RRT_dim1_traj.back(), goal_threads[thread_ind], rrt_dim1_endtogoal_interpolate);
    solveSQP(rrt_dim1_endtogoal_interpolate, rrt_dim1_endtogoal_sqp_traj, rrt_dim1_endtogoal_sqp_controls, sqp_namestring);
    wrap_controls_extra_vector(rrt_dim1_endtogoal_sqp_controls, rrt_dim1_endtogoal_sqp_controls_wrapped);
    closedLoopLinearizationController(rrt_dim1_endtogoal_sqp_traj, rrt_dim1_endtogoal_sqp_controls_wrapped, RRT_dim1_SQP_closedloop_onlylast_traj);
    RRT_dim1_SQP_closedloop_onlylast_recorder.add_threads_to_list(RRT_dim1_traj);
    RRT_dim1_SQP_closedloop_onlylast_recorder.add_threads_to_list(RRT_dim1_SQP_closedloop_onlylast_traj);
    //deleteAllThreads(current_traj);

    results_file << "RRT_dim1_PLANNER_SQP_CLOSEDLOOP_ONLY_LAST," 
      << cost_metric(RRT_dim1_SQP_closedloop_onlylast_traj.back(), goal_threads[thread_ind])
      << ","
      << RRT_dim1_PLANNER_TIME + timer.elapsed()
      << endl;


if (start_threads[thread_ind]->num_pieces() > 15)
{

/***********************************************
 * DIM REDUCE 2
***********************************************/ 
    timer.restart(); 
    //generate RRT_dim2
    vector<vector<VectorXd> > RRT_dim2_controls;
    RRTPlanner(start_threads[thread_ind], goal_threads[thread_ind], 2, RRT_dim2_traj, RRT_dim2_controls);
    RRT_dim2_recorder.add_threads_to_list(RRT_dim2_traj);
    double RRT_dim2_PLANNER_TIME = timer.elapsed(); 
    results_file << "RRT_dim2_PLANNER," 
      << cost_metric(RRT_dim2_traj.back(), goal_threads[thread_ind])
      << ","
      << RRT_dim2_PLANNER_TIME
      << endl;

    
    //add goal to RRT_dim2 for sqp
    vector<Thread*> RRT_dim2_traj_togoal;
    traj_subsampling(RRT_dim2_traj, RRT_dim2_traj_togoal);
    RRT_dim2_traj_togoal.push_back(new Thread(*goal_threads[thread_ind]));
  
    timer.restart();
    //smooth with SQP, openloop
    vector<Thread*> rrt_dim2_sqp_traj;
    vector<VectorXd> rrt_dim2_sqp_controls;
    vector<vector<VectorXd> > rrt_dim2_sqp_controls_wrapped;
    solveSQP(RRT_dim2_traj_togoal, rrt_dim2_sqp_traj, rrt_dim2_sqp_controls, sqp_namestring);
    wrap_controls_extra_vector(rrt_dim2_sqp_controls, rrt_dim2_sqp_controls_wrapped);
    double RRT_dim2_SQP_SMOOTHING_TIME = timer.elapsed(); 

    timer.restart();
    //playback sqp openloop
    openLoopController(rrt_dim2_sqp_traj, rrt_dim2_sqp_controls, RRT_dim2_SQP_openloop_traj);
    RRT_dim2_SQP_openloop_recorder.add_threads_to_list(RRT_dim2_SQP_openloop_traj);

    results_file << "RRT_dim2_PLANNER_SQP_OPENLOOP_SMOOTHER," 
      << cost_metric(RRT_dim2_SQP_openloop_traj.back(), goal_threads[thread_ind])
      << ","
      << RRT_dim2_PLANNER_TIME + RRT_dim2_SQP_SMOOTHING_TIME + timer.elapsed()
      << endl;
    //deleteAllThreads(current_traj);

    timer.restart(); 
    //playback sqp closedloop
    closedLoopLinearizationController(rrt_dim2_sqp_traj, rrt_dim2_sqp_controls_wrapped, RRT_dim2_SQP_closedloop_traj);
    RRT_dim2_SQP_closedloop_recorder.add_threads_to_list(RRT_dim2_SQP_closedloop_traj);
    //deleteAllThreads(current_traj);

    results_file << "RRT_dim2_PLANNER_SQP_CLOSEDLOOP_SMOOTHER," 
      << cost_metric(RRT_dim2_SQP_closedloop_traj.back(), goal_threads[thread_ind])
      << ","
      << RRT_dim2_PLANNER_TIME + RRT_dim2_SQP_SMOOTHING_TIME + timer.elapsed()
      << endl;

    timer.restart(); 
    //solve sqp from end of rrt_dim2
    vector<Thread*> rrt_dim2_endtogoal_interpolate;
    vector<Thread*> rrt_dim2_endtogoal_sqp_traj;
    vector<VectorXd> rrt_dim2_endtogoal_sqp_controls;
    vector<vector<VectorXd> > rrt_dim2_endtogoal_sqp_controls_wrapped;
    interpolatePointsTrajectory(RRT_dim2_traj.back(), goal_threads[thread_ind], rrt_dim2_endtogoal_interpolate);
    solveSQP(rrt_dim2_endtogoal_interpolate, rrt_dim2_endtogoal_sqp_traj, rrt_dim2_endtogoal_sqp_controls, sqp_namestring);
    wrap_controls_extra_vector(rrt_dim2_endtogoal_sqp_controls, rrt_dim2_endtogoal_sqp_controls_wrapped);
    closedLoopLinearizationController(rrt_dim2_endtogoal_sqp_traj, rrt_dim2_endtogoal_sqp_controls_wrapped, RRT_dim2_SQP_closedloop_onlylast_traj);
    RRT_dim2_SQP_closedloop_onlylast_recorder.add_threads_to_list(RRT_dim2_traj);
    RRT_dim2_SQP_closedloop_onlylast_recorder.add_threads_to_list(RRT_dim2_SQP_closedloop_onlylast_traj);
    //deleteAllThreads(current_traj);

    results_file << "RRT_dim2_PLANNER_SQP_CLOSEDLOOP_ONLY_LAST," 
      << cost_metric(RRT_dim2_SQP_closedloop_onlylast_traj.back(), goal_threads[thread_ind])
      << ","
      << RRT_dim2_PLANNER_TIME + timer.elapsed()
      << endl;

}


    
  


    
    //delete unneeded?
    //deleteAllThreads(interpolated_ends);
    /*deleteAllThreads(interpolated_points);
    deleteAllThreads(sqp_points);
    deleteAllThreads(rrt_sqp_traj);
    deleteAllThreads(rrt_endtogoal_interpolate);
    deleteAllThreads(rrt_endtogoal_sqp_traj);
*/
    results_file.close();
    linearize_only_recorder.write_threads_to_file();
    //interpolate_end_and_linear_recorder.write_threads_to_file();
    interpolate_point_and_linearize_recorder.write_threads_to_file();
    sqp_openloop_recorder.write_threads_to_file();
    sqp_closedloop_recorder.write_threads_to_file();
    RRT_recorder.write_threads_to_file();
    RRT_SQP_openloop_recorder.write_threads_to_file();
    RRT_SQP_closedloop_recorder.write_threads_to_file();
    RRT_SQP_closedloop_onlylast_recorder.write_threads_to_file();
    RRT_dim1_recorder.write_threads_to_file();
    RRT_dim1_SQP_openloop_recorder.write_threads_to_file();
    RRT_dim1_SQP_closedloop_recorder.write_threads_to_file();
    RRT_dim1_SQP_closedloop_onlylast_recorder.write_threads_to_file();
    RRT_dim2_recorder.write_threads_to_file();
    RRT_dim2_SQP_openloop_recorder.write_threads_to_file();
    RRT_dim2_SQP_closedloop_recorder.write_threads_to_file();
    RRT_dim2_SQP_closedloop_onlylast_recorder.write_threads_to_file();


  
  } 


}


void deleteAllThreads(vector<Thread*>& toDelete)
{
  /*for (int i=0; i < toDelete.size(); i++)
  {
    delete toDelete[i];
  }*/
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


void Copy_Threads(const vector<Thread*>& to_copy, vector<Thread*>& copy)
{
  copy.resize(to_copy.size());
  for (int i=0; i < to_copy.size(); i++)
  {
    copy[i] = new Thread(*to_copy[i]);
  }
}




  






  






  



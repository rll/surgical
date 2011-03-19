#include "try_energy_params_move_noniso.h"
#include <time.h>
#include <sys/time.h>


int main (int argc, char * argv[])
{
    //ofstream fout("/dev/null");
    //cout.rdbuf(fout.rdbuf());
	if (strcmp(argv[ARG_EVAL_MODE], "evalGrad") == 0)
	{
			estimationMode = EVALGRAD;
	}	else if (strcmp(argv[ARG_EVAL_MODE], "oneStep") == 0) {
			estimationMode = ONESTEPPROJECT;
	}	else if (strcmp(argv[ARG_EVAL_MODE], "fullMini") == 0) {
			estimationMode = FULLMINIMIZATION;
	}	else	{
		std::cerr << "incorrect evaluation mode" << std::endl;
		exit(0);
	}


	

  //set the directory with thread information
  /*if (argc > 2 && !strcmp(argv[1], "est"))
  {
    sprintf(opt_info_dir, "%s", argv[2]);
  } else {
    sprintf(opt_info_dir, "config");
  }
  */
  sprintf(opt_info_dir, "config");

  char thread_file[256];
  sprintf(thread_file, "%s/%s", opt_info_dir, argv[ARG_PARAM_FILE]); 
  Trajectory_Reader traj_reader(thread_file);
  traj_reader.read_threads_from_file();
  
  //all_threads.resize(traj_reader.get_all_threads().size());
	all_threads.resize(0);
  scores_each_thread.resize(traj_reader.get_all_threads().size());
  scores_each_thread_norm.resize(traj_reader.get_all_threads().size());
  scores_material_frame_thread.resize(traj_reader.get_all_threads().size());
  scores_twist_thread.resize(traj_reader.get_all_threads().size());
  orig_points.resize(traj_reader.get_all_threads().size());
  orig_twist_angles.resize(traj_reader.get_all_threads().size());
  orig_material_frames.resize(traj_reader.get_all_threads().size());
  for (int i=0; i < traj_reader.get_all_threads().size(); i++)
  {
		all_threads.push_back(traj_reader.get_all_threads()[i]);
		all_threads.back().get_thread_data(orig_points[i], orig_twist_angles[i], orig_material_frames[i]);
//    all_threads[i] = traj_reader.get_all_threads()[i];
//		all_threads[i].get_thread_data(orig_points[i], orig_twist_angles[i]);
  }
	
	double min_twist = atof(argv[ARG_MIN_TWIST]);
	double iter_twist = atof(argv[ARG_ITER_TWIST]);
	double max_twist = atof(argv[ARG_MAX_TWIST]);
	double min_bend_m1 = atof(argv[ARG_MIN_BEND_M1]);
	double iter_bend_m1 = atof(argv[ARG_ITER_BEND_M1]);
	double max_bend_m1 = atof(argv[ARG_MAX_BEND_M1]);
	double min_bend_m2 = atof(argv[ARG_MIN_BEND_M2]);
	double iter_bend_m2 = atof(argv[ARG_ITER_BEND_M2]);
	double max_bend_m2 = atof(argv[ARG_MAX_BEND_M2]);
	double min_bend_joint = atof(argv[ARG_MIN_BEND_JOINT]);
	double iter_bend_joint = atof(argv[ARG_ITER_BEND_JOINT]);
	double max_bend_joint = atof(argv[ARG_MAX_BEND_JOINT]);
    double num_movements = atof(argv[ARG_NUM_MOVEMENTS]);

  scores_two_points_thread.resize(traj_reader.get_all_threads().size() * (num_movements+1));
  scores_two_points_thread_norm.resize(traj_reader.get_all_threads().size()*(num_movements+1));
  sprintf(data_write_base, "results/%s_%s_%f_%f_%f_%f_%f_%f_%f_%f_%f_%f_%f_%f_", argv[ARG_PARAM_FILE], argv[ARG_EVAL_MODE], min_twist, iter_twist, max_twist, min_bend_m1, iter_bend_m1, max_bend_m1, min_bend_m2, iter_bend_m2, max_bend_m2, min_bend_joint, iter_bend_joint, max_bend_joint);
  sprintf(data_write_base_norm, "results/norm_%s_%s_%f_%f_%f_%f_%f_%f_%f_%f_%f_%f_%f_%f_", argv[ARG_PARAM_FILE], argv[ARG_EVAL_MODE], min_twist, iter_twist, max_twist, min_bend_m1, iter_bend_m1, max_bend_m1, min_bend_m2, iter_bend_m2, max_bend_m2, min_bend_joint, iter_bend_joint, max_bend_joint);

	char results_filename[256];
	sprintf(results_filename, "%sresults.txt", data_write_base);
	std::ofstream results_out;
	results_out.precision(10);
	results_out.open(results_filename);

	char results_filename_norm[256];
	sprintf(results_filename_norm, "%sresults.txt", data_write_base_norm);
	std::ofstream results_out_norm;
	results_out_norm.precision(10);
	results_out_norm.open(results_filename_norm);


	vector<double> params(5);
	double grav=0.0001;
	for (double bend_m1=min_bend_m1; bend_m1 <= max_bend_m1; bend_m1+=iter_bend_m1)
	{
        for (double bend_m2=min_bend_m2; bend_m2 <= max_bend_m2; bend_m2+=iter_bend_m2)
        {
            for (double bend_joint=min_bend_joint; bend_joint <= max_bend_joint; bend_joint+=iter_bend_joint)
            {
                for (double twist=min_twist; twist <= max_twist; twist+=iter_twist)
                {

                    //params[0] = pow(2, bend);
                    //params[1] = pow(2, twist)*pow(2, bend);
                    /*params[0] = pow(2,bend_m1); //m1
                    params[1] = pow(2,bend_m2)*pow(2,bend_m1); //m2
                    params[2] = bend_joint; //joint
                    params[3] = pow(2,twist)*sqrt(pow(2,bend_m2)*pow(2,bend_m1)); //twist
                    params[4] = grav; //grav
                    */

                    params[0] = pow(2,twist)*pow(2,bend_m1); //m1
                    params[1] = pow(2,twist)*pow(2,bend_m1)*pow(2,bend_m2); //m2
                    params[2] = bend_joint; //joint
                    params[3] = pow(2,twist); //twist 
                    params[4] = grav; //grav
                    double bend_m1_set = params[0];
                    double bend_m2_set = params[1];
                    double bend_joint_set = params[2];
                    double twist_set = params[3];
                    double grav_set = params[4];

                    double this_score;

                    if (estimationMode == FULLMINIMIZATION)
                        this_score = full_minimization(params, num_movements);

                    //results_out << pow(2,bend) << " " << pow(2, twist)*pow(2,bend) << " " << grav << " " << this_score;
                    //results_out << twist << " " << bend_m1 << " " << bend_m2 << " " << bend_joint << " " << grav << " " << this_score;
                    results_out << twist_set << " " << bend_m1_set << " " << bend_m2_set << " " << bend_joint_set << " " << grav_set << " " << this_score;
                    results_out_norm << twist_set << " " << bend_m1_set << " " << bend_m2_set << " " << bend_joint_set << " " << grav_set << " " << this_score;
                    for (int thread_ind=0; thread_ind < scores_each_thread.size(); thread_ind++)
                    {
                        results_out << " " << scores_each_thread[thread_ind];
                        results_out_norm << " " << scores_each_thread_norm[thread_ind];
                    }
                    for (int thread_ind=0; thread_ind < scores_material_frame_thread.size(); thread_ind++)
                    {
                        //results_out << " " << scores_material_frame_thread[thread_ind];
                    }

                    for (int thread_ind=0; thread_ind < scores_twist_thread.size(); thread_ind++)
                    {
                        //results_out << " " << scores_twist_thread[thread_ind];
                    }
                    for (int thread_ind=0; thread_ind < scores_two_points_thread.size(); thread_ind++)
                    {
                        results_out << " " << scores_two_points_thread[thread_ind];
                        results_out_norm << " " << scores_two_points_thread_norm[thread_ind];
                    }
                    results_out << "\n";
                    results_out_norm << "\n";
                    //std::cout << pow(2,bend) << " " << pow(2, twist)*pow(2,bend) << " " << grav << " " << this_score << "\n";
                    //std::cout << twist << " " << bend_m1 << " " << bend_m2 << " " << bend_joint << " " << grav << " " << this_score << "\n";
                    std::cout << twist_set << " " << bend_m1_set << " " << bend_m2_set << " " << bend_joint_set << " " << grav_set << " " << this_score << "\n";
                }
            }
        }
    }

	results_out.close();
	results_out_norm.close();

  std::cout << "finished " << std::endl;
}

vector<Thread> construct_thread_sequence_forward(int startIndex, int numSeq, vector<Thread> &result) {
    result.push_back(all_threads[startIndex]); 
    for (int i = 0; i < numSeq; i++) {

        if ((startIndex - NUM_EXP_ANG + 1) % NUM_EXP_ANG == 0) {
            break;
        } else {
            startIndex += 1;
            cout << "Adding index " << startIndex << endl;
            result.push_back(all_threads[startIndex]);
        }

    }
    return result;
}


double full_minimization(vector<double>& coeffs, int num_movements)
{
  #pragma omp parallel for num_threads(NUM_PARALLEL)
	for (int i=0; i < all_threads.size(); i++)
	{
        cout << "Start index " << i << endl;
        vector<Thread> thread_seq;
        thread_seq.resize(0);
        thread_seq = construct_thread_sequence_forward(i, num_movements, thread_seq);

        vector<Vector3d> points, goal_points;
		vector<double> twist_angles, goal_twist_angles;
        vector<Matrix3d> material_frames, goal_material_frames;

        Thread runningThread = thread_seq[0];

        Matrix2d bend_coeffs = Matrix2d(2,2);
        bend_coeffs << coeffs[0], coeffs[2],
                       coeffs[2], coeffs[1];
		runningThread.set_coeffs_normalized(bend_coeffs, coeffs[3], coeffs[4]);
        // scores_two_points_thread = scores_each_movement in iso case
        // scores_each_thread is aggregate movement error over all motions per thread 
        for (int seq = 0; seq < num_movements+1; seq++) {
            scores_two_points_thread[(num_movements+1)*i +seq] = 0.0; 
            scores_each_thread[i] = 0.0;
            scores_two_points_thread_norm[(num_movements+1)*i +seq] = 0.0; 
            scores_each_thread_norm[i] = 0.0;
            if (seq < thread_seq.size()) {
                Thread intermediateConfig = thread_seq[seq];
                runningThread.match_start_and_end_constraints(intermediateConfig, THREAD_MOVEMENT_STEPS);
                runningThread.get_thread_data(points, twist_angles, material_frames);
                intermediateConfig.get_thread_data(goal_points, goal_twist_angles, goal_material_frames);
                for (int j = 0; j < points.size(); j++) {
                    scores_each_thread[i] += pow((points[j] - goal_points[j]).norm(),2);
                    scores_each_thread_norm[i] += (points[j] - goal_points[j]).norm();
                    if (j > 0 && j < points.size() - 1) {
                        Vector3d goal_top = goal_points[j] + RIBBON_HALF_WIDTH * goal_material_frames[j].col(1).normalized();
                        Vector3d goal_bot = goal_points[j] - RIBBON_HALF_WIDTH * goal_material_frames[j].col(1).normalized();
                        Vector3d current_top = points[j] + RIBBON_HALF_WIDTH * material_frames[j].col(1).normalized();
                        Vector3d current_bot = points[j] - RIBBON_HALF_WIDTH * material_frames[j].col(1).normalized();

                        scores_two_points_thread[(num_movements+1)*i + seq] += (goal_top - current_top).squaredNorm();
                        scores_two_points_thread[(num_movements+1)*i + seq] += (goal_bot - current_bot).squaredNorm();
                        scores_two_points_thread_norm[(num_movements+1)*i + seq] += (goal_top - current_top).norm();
                        scores_two_points_thread_norm[(num_movements+1)*i + seq] += (goal_bot - current_bot).norm();
                    }
                }
                scores_two_points_thread[(num_movements+1)*i + seq] = sqrt(scores_two_points_thread[(num_movements+1)*i + seq]);
                scores_each_thread[i] = sqrt(scores_each_thread[i]);
            }
        }
    }

	double fx = 0.0;

	for (int i=0; i < all_threads.size(); i++)
	{
		fx += scores_each_thread[i];
        fx += scores_two_points_thread[i];
	}

	fx /= (2.0*(double)scores_each_thread.size());

	return fx;
}


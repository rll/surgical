#include "try_energy_params_move.h"
#include <time.h>
#include <sys/time.h>

int main (int argc, char * argv[])
{
  //ofstream fout("/dev/null");
  //cout.rdbuf(fout.rdbuf());
	if (strcmp(argv[ARG_EVAL_MODE], "fullMini") == 0) {
			estimationMode = FULLMINIMIZATION;
	}	else	{
		std::cerr << "incorrect evaluation mode" << std::endl;
		exit(0);
	}

  sprintf(opt_info_dir, "config");

  char thread_file[256];
  sprintf(thread_file, "%s/%s", opt_info_dir, argv[ARG_PARAM_FILE]); 
  Trajectory_Reader traj_reader(thread_file);
  traj_reader.read_threads_from_file();
  
  //all_threads.resize(traj_reader.get_all_threads().size());
	all_threads.resize(0);
  scores_each_thread.resize(traj_reader.get_all_threads().size());
  scores_each_thread_norm.resize(traj_reader.get_all_threads().size());
  orig_points.resize(traj_reader.get_all_threads().size());
	orig_twist_angles.resize(traj_reader.get_all_threads().size());
  for (int i=0; i < traj_reader.get_all_threads().size(); i++)
  {
		all_threads.push_back(traj_reader.get_all_threads()[i]);
		all_threads.back().get_thread_data(orig_points[i], orig_twist_angles[i]);
//    all_threads[i] = traj_reader.get_all_threads()[i];
//		all_threads[i].get_thread_data(orig_points[i], orig_twist_angles[i]);
  }
	
	double min_twist = atof(argv[ARG_MIN_TWIST]);
	double iter_twist = atof(argv[ARG_ITER_TWIST]);
	double max_twist = atof(argv[ARG_MAX_TWIST]);
	double min_bend = atof(argv[ARG_MIN_BEND]);
	double iter_bend = atof(argv[ARG_ITER_BEND]);
	double max_bend = atof(argv[ARG_MAX_BEND]);
    double num_movements = atof(argv[ARG_NUM_MOVEMENTS]);


    scores_each_movement.resize(traj_reader.get_all_threads().size() * (num_movements+1));
    scores_each_movement_norm.resize(traj_reader.get_all_threads().size()*(num_movements+1));


  sprintf(data_write_base, "results/%s_%s_%f_%f_%f_%f_%f_%f_%f_", argv[ARG_PARAM_FILE], argv[ARG_EVAL_MODE], min_twist, iter_twist, max_twist, min_bend, iter_bend, max_bend, num_movements);

  sprintf(data_write_base_norm, "results/norm_%s_%s_%f_%f_%f_%f_%f_%f_%f_", argv[ARG_PARAM_FILE], argv[ARG_EVAL_MODE], min_twist, iter_twist, max_twist, min_bend, iter_bend, max_bend, num_movements);
	char results_filename[256];
    char results_filename_norm[256];
	sprintf(results_filename, "%sresults.txt", data_write_base);
    sprintf(results_filename_norm, "%sresults.txt", data_write_base_norm);
	std::ofstream results_out;
    std::ofstream results_out_norm;
	results_out.precision(10);
	results_out.open(results_filename);

    results_out_norm.precision(10);
    results_out_norm.open(results_filename_norm);

	vector<double> params(3);
	double grav=0.0001;
	for (double bend=min_bend; bend <= max_bend; bend+=iter_bend)
	{
		for (double twist=min_twist; twist <= max_twist; twist+=iter_twist)
		{

            double bend_param = pow(2,bend);
            double twist_param = pow(2, twist)*pow(2,bend);
            //double bend_param = bend;
            //double twist_param = twist;
            double grav_param = grav;

            params[0] = bend_param;
            params[1] = twist_param;
            //params[0] = bend;
            //params[1] = twist;
            params[2] = grav_param;

            double this_score;


			if (estimationMode == FULLMINIMIZATION)
				this_score = full_minimization(params, num_movements);
			
			results_out <<bend_param << " " <<twist_param << " " << grav_param << " " << this_score << " " << num_movements;
			results_out_norm <<bend_param << " " <<twist_param << " " << grav_param << " " << this_score << " " << num_movements;
			for (int thread_ind=0; thread_ind < scores_each_thread.size(); thread_ind++)
			{
				//results_out << " " << scores_each_thread[thread_ind];
				//results_out_norm << " " << scores_each_thread_norm[thread_ind];
			}
			for (int thread_ind=0; thread_ind < scores_each_movement.size(); thread_ind++)
			{
				results_out << " " << scores_each_movement[thread_ind];
				results_out_norm << " " << scores_each_movement_norm[thread_ind];
			}
			results_out << "\n";
            results_out_norm << "\n";
      
			std::cout << bend_param << " " << twist_param << " " << grav_param << " " << this_score << " " << num_movements << "\n";
		}
	}

	results_out.close();
    results_out_norm.close();

  std::cout << "finished " << std::endl;
}

vector<Thread> construct_thread_sequence(int goalIndex, int numSeq, vector<Thread> &result) {
    int startIndex = 0;
    if(goalIndex == 0) {
        startIndex = 0;
    } else if (goalIndex % NUM_EXP_ANG == 0) {
        startIndex = ((goalIndex / NUM_EXP_ANG) - 1)*NUM_EXP_ANG;
    } else {
        startIndex = goalIndex-1;
    }
//   cout << "Adding index " << startIndex << endl;
    result.push_back(all_threads[startIndex]);
    if(numSeq != 1) {
        construct_thread_sequence(startIndex, numSeq - 1, result);
    } else {
        //reverse list and return
        vector<Thread> reverse_result;
        reverse_result.resize(0);
        int numElem = result.size();
        for(int i = 0; i < numElem; i++) { 
            reverse_result.push_back(result.back());
            result.pop_back();
        }
        result = reverse_result;
        return reverse_result;
    }

    return result;

}

vector<Thread> construct_thread_sequence_forward(int startIndex, int numSeq, vector<Thread> &result) {
    result.push_back(all_threads[startIndex]); 
    for (int i = 0; i < numSeq; i++) {

        if ((startIndex - NUM_EXP_ANG + 1) % NUM_EXP_ANG == 0) {
            break;
        } else {
            startIndex += 1;
            //cout << "Adding index " << startIndex << endl;
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
        vector<Thread> thread_seq;
        thread_seq.resize(0);
        //cout << "Start index: " << i << endl;
        thread_seq = construct_thread_sequence_forward(i, num_movements, thread_seq);
        vector<Vector3d> points, goal_points;
		vector<double> twist_angles, goal_twist_angles;
        
        Thread runningThread = thread_seq[0];
        runningThread.set_coeffs_normalized(coeffs[0], coeffs[1], coeffs[2]);
        //runningThread.minimize_energy();
        for (int seq = 0; seq < num_movements+1; seq++) {
           scores_each_movement[(num_movements+1)*i + seq] = 0.0;
           scores_each_movement_norm[(num_movements+1)*i +seq] =0.0;
           scores_each_thread[i] = 0.0;
           scores_each_thread_norm[i] = 0.0;
            if (seq < thread_seq.size()) {
                Thread intermediateConfig = thread_seq[seq];
                runningThread.match_start_and_end_constraints(intermediateConfig, THREAD_MOVEMENT_STEPS);
                runningThread.get_thread_data(points, twist_angles);
                intermediateConfig.get_thread_data(goal_points, goal_twist_angles);
                for (int j = 0; j < points.size(); j++) {
                    scores_each_movement[(num_movements+1)*i + seq] += pow((points[j] - goal_points[j]).norm(),2);
                    scores_each_movement_norm[(num_movements+1)*i + seq] += (points[j] - goal_points[j]).norm();
                }
                scores_each_movement[(num_movements+1)*i + seq] = sqrt(scores_each_movement[(num_movements+1)*i + seq]);
                scores_each_thread[i] = sqrt(scores_each_movement[(num_movements+1)*i + seq]);
                scores_each_thread_norm[i] = scores_each_movement_norm[(num_movements+1)*i + seq];
            }
        }
	}

	double fx = 0.0;

	for (int i=0; i < all_threads.size(); i++)
	{
		fx += scores_each_thread[i];
	}

	fx /= (double)(NUM_EXP_ANG*NUM_EXP-NUM_EXP*num_movements);

	return fx;
}


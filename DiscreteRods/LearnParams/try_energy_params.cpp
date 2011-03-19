#include "try_energy_params.h"
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
  angle_scores_each_thread.resize(traj_reader.get_all_threads().size());
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

  sprintf(data_write_base, "results/%s_%s_%f_%f_%f_%f_%f_%f_", argv[ARG_PARAM_FILE], argv[ARG_EVAL_MODE], min_twist, iter_twist, max_twist, min_bend, iter_bend, max_bend);

	char results_filename[256];
	sprintf(results_filename, "%sresults.txt", data_write_base);
	std::ofstream results_out;
	results_out.precision(10);
	results_out.open(results_filename);

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


			if (estimationMode == EVALGRAD)
				this_score = evalEnergyGradient(params);
			else if (estimationMode == ONESTEPPROJECT)
				this_score = oneStepDistance(params);
			else if (estimationMode == FULLMINIMIZATION)
				this_score = full_minimization(params);
			
			results_out <<bend_param << " " <<twist_param << " " << grav_param << " " << this_score;
			for (int thread_ind=0; thread_ind < scores_each_thread.size(); thread_ind++)
			{
				results_out << " " << scores_each_thread[thread_ind];
			}
			results_out << "\n";
      
			//std::cout << pow(2,bend) << " " << pow(2, twist)*pow(2,bend) << " " << grav << " " << this_score << "\n";
			std::cout << bend_param << " " << twist_param << " " << grav_param << " " << this_score << "\n";
		}
	}

	results_out.close();

  std::cout << "finished " << std::endl;
}

double evalEnergyGradient(vector<double>& coeffs)
{
  double fx = 0.0;

  vector<Vector3d> vertex_gradients;
  
	for (int i=0; i < all_threads.size(); i++)
  {
    scores_each_thread[i] = 0.0;
    vertex_gradients.resize(all_threads[i].num_pieces());
		all_threads[i].set_coeffs_normalized(coeffs[0], coeffs[1], coeffs[2]);

		all_threads[i].calculate_gradient_vertices(vertex_gradients);

    for (int j=2; j < vertex_gradients.size()-2; j++)
    {
      scores_each_thread[i] += vertex_gradients[j].norm();
    }
    scores_each_thread[i] /= (double)((int)vertex_gradients.size()-4);

    fx += scores_each_thread[i];
  }

  fx /= (double)all_threads.size();

	return fx;

}

double oneStepDistance(vector<double>& coeffs)
{
  #pragma omp parallel for num_threads(NUM_PARALLEL)
	for (int i=0; i < all_threads.size(); i++)
	{
    double step_size = 0.01;
		vector<Vector3d> points;
		vector<double> twist_angles;
		Thread toProcess(all_threads[i]);	//must make a copy!
		toProcess.set_coeffs_normalized(coeffs[0], coeffs[1], coeffs[2]);

		toProcess.one_step_project(step_size);
		toProcess.get_thread_data(points, twist_angles);

		scores_each_thread[i] = 0.0;
		for (int j=0; j < points.size(); j++)
		{
			scores_each_thread[i] += (pow((points[j] - orig_points[i][j]).norm(),2));
		}
		scores_each_thread[i] = sqrt(scores_each_thread[i])/step_size;
	}

	
	double fx = 0.0;
	for (int i=0; i < all_threads.size(); i++)
	{
		fx += scores_each_thread[i];
	}

	fx /= (double)scores_each_thread.size();

	return fx;



}


double full_minimization(vector<double>& coeffs)
{
  #pragma omp parallel for num_threads(NUM_PARALLEL)
	for (int i=0; i < all_threads.size(); i++)
	{
		vector<Vector3d> points;
		vector<double> twist_angles;
		Thread toProcess(all_threads[i]);	//must make a copy!
		toProcess.set_coeffs_normalized(coeffs[0], coeffs[1], coeffs[2]);

		toProcess.minimize_energy();
		toProcess.get_thread_data(points, twist_angles);


		scores_each_thread[i] = 0.0;
		for (int j=0; j < points.size(); j++)
		{
			scores_each_thread[i] += (points[j] - orig_points[i][j]).squaredNorm();
		}
		scores_each_thread[i] = sqrt(scores_each_thread[i]);
    angle_scores_each_thread[i] = abs(twist_angles[twist_angles.size()-2] - orig_twist_angles[i][twist_angles.size()-2]);
		//std::cout << "point score: " << scores_each_thread[i] << std::endl;
    //std::cout << "angle score: " << angle_scores_each_thread[i] << std::endl;
	}

	
	double fx = 0.0;

	for (int i=0; i < all_threads.size(); i++)
	{
		//fx += scores_each_thread[i];
    fx += angle_scores_each_thread[i];
	}

	fx /= (double)scores_each_thread.size();

	return fx;
}


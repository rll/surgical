#include "try_energy_params_noniso.h"
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
  scores_material_frame_thread.resize(traj_reader.get_all_threads().size());
  scores_twist_thread.resize(traj_reader.get_all_threads().size());
  scores_two_points_thread.resize(traj_reader.get_all_threads().size());
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
 


  sprintf(data_write_base, "results/%s_%s_%f_%f_%f_%f_%f_%f_%f_%f_%f_%f_%f_%f_", argv[ARG_PARAM_FILE], argv[ARG_EVAL_MODE], min_twist, iter_twist, max_twist, min_bend_m1, iter_bend_m1, max_bend_m1, min_bend_m2, iter_bend_m2, max_bend_m2, min_bend_joint, iter_bend_joint, max_bend_joint);

	char results_filename[256];
	sprintf(results_filename, "%sresults.txt", data_write_base);
	std::ofstream results_out;
	results_out.precision(10);
	results_out.open(results_filename);

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

                    if (estimationMode == EVALGRAD)
                        this_score = evalEnergyGradient(params);
                    else if (estimationMode == ONESTEPPROJECT)
                        this_score = oneStepDistance(params);
                    else if (estimationMode == FULLMINIMIZATION)
                        this_score = full_minimization(params);

                    //results_out << pow(2,bend) << " " << pow(2, twist)*pow(2,bend) << " " << grav << " " << this_score;
                    //results_out << twist << " " << bend_m1 << " " << bend_m2 << " " << bend_joint << " " << grav << " " << this_score;
                    results_out << twist_set << " " << bend_m1_set << " " << bend_m2_set << " " << bend_joint_set << " " << grav_set << " " << this_score;
                    for (int thread_ind=0; thread_ind < scores_each_thread.size(); thread_ind++)
                    {
                        results_out << " " << scores_each_thread[thread_ind];
                    }
                    for (int thread_ind=0; thread_ind < scores_material_frame_thread.size(); thread_ind++)
                    {
                        results_out << " " << scores_material_frame_thread[thread_ind];
                    }

                    for (int thread_ind=0; thread_ind < scores_twist_thread.size(); thread_ind++)
                    {
                        results_out << " " << scores_twist_thread[thread_ind];
                    }
                    for (int thread_ind=0; thread_ind < scores_two_points_thread.size(); thread_ind++)
                    {
                        results_out << " " << scores_two_points_thread[thread_ind];
                    }
                    results_out << "\n";
                    //std::cout << pow(2,bend) << " " << pow(2, twist)*pow(2,bend) << " " << grav << " " << this_score << "\n";
                    //std::cout << twist << " " << bend_m1 << " " << bend_m2 << " " << bend_joint << " " << grav << " " << this_score << "\n";
                    std::cout << twist_set << " " << bend_m1_set << " " << bend_m2_set << " " << bend_joint_set << " " << grav_set << " " << this_score << "\n";
                }
            }
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
    Matrix2d bend_coeffs = Matrix2d(2,2);
    bend_coeffs << coeffs[0], coeffs[2],
                   coeffs[2], coeffs[1];
    all_threads[i].set_coeffs_normalized(bend_coeffs, coeffs[3], coeffs[4]);

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
        vector<Vector3d> points;
		vector<double> twist_angles;
		Thread toProcess(all_threads[i]);	//must make a copy!
        Matrix2d bend_coeffs = Matrix2d(2,2);
        bend_coeffs << coeffs[0], coeffs[2],
                       coeffs[2], coeffs[1];
		toProcess.set_coeffs_normalized(bend_coeffs, coeffs[3], coeffs[4]);

		toProcess.one_step_project();
		toProcess.get_thread_data(points, twist_angles);

		scores_each_thread[i] = 0.0;
		for (int j=0; j < points.size(); j++)
		{
			scores_each_thread[i] += pow((points[j] - orig_points[i][j]).norm(),2);
        }
		scores_each_thread[i] = sqrt(scores_each_thread[i]);
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
        Matrix2d bend_coeffs = Matrix2d(2,2);
        bend_coeffs << coeffs[0], coeffs[2],
                       coeffs[2], coeffs[1];
		toProcess.set_coeffs_normalized(bend_coeffs, coeffs[3], coeffs[4]);

		toProcess.minimize_energy();
        vector<Matrix3d> material_frames;
		toProcess.get_thread_data(points, twist_angles, material_frames);


		scores_each_thread[i] = 0.0;
        scores_material_frame_thread[i] = 0.0;
        scores_twist_thread[i] = 0.0;
        scores_two_points_thread[i] = 0.0;
		for (int j=0; j < points.size(); j++)
		{
			scores_each_thread[i] += pow((points[j] - orig_points[i][j]).norm(),2);
            if (j > 0 && j < points.size()-1) {
                scores_material_frame_thread[i] += pow(angle_mismatch(orig_material_frames[i][j],material_frames[j]), 2);
                scores_twist_thread[i] += pow(twist_angles[j] - orig_twist_angles[i][j], 2);
           
                Vector3d orig_top = orig_points[i][j] + RIBBON_HALF_WIDTH * orig_material_frames[i][j].col(1).normalized();
                Vector3d orig_bot = orig_points[i][j] - RIBBON_HALF_WIDTH * orig_material_frames[i][j].col(1).normalized();
                Vector3d thread_top = points[j] + RIBBON_HALF_WIDTH * material_frames[j].col(1).normalized();
                Vector3d thread_bot = points[j] - RIBBON_HALF_WIDTH * material_frames[j].col(1).normalized();

                scores_two_points_thread[i] += (orig_top - thread_top).squaredNorm();
                scores_two_points_thread[i] += (orig_bot - thread_bot).squaredNorm();
            }
		}
		scores_each_thread[i] = sqrt(scores_each_thread[i]);
        scores_material_frame_thread[i] = sqrt(scores_material_frame_thread[i]);
        scores_twist_thread[i] = sqrt(scores_twist_thread[i]);
        scores_two_points_thread[i] = sqrt(scores_two_points_thread[i]);
		//std::cout << scores_each_thread[i] << std::endl;
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


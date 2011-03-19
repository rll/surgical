#include "estimate_thread_params.h"
#include <time.h>
#include <sys/time.h>


#include "NLF.h"
#include "OptQNewton.h"
#include "NLP.h"
#include "GenSet.h"
#include "OptPDS.h"



int main (int argc, char * argv[])
{
  //set the directory with thread information
  /*if (argc > 2 && !strcmp(argv[1], "est"))
  {
    sprintf(opt_info_dir, "%s", argv[2]);
  } else {
    sprintf(opt_info_dir, "config");
  }
  */
  sprintf(opt_info_dir, "config");
  sprintf(data_write_base, "config/results/%s_", argv[ARG_PARAMS_OUT_BASE]);



  double* params = new double[3];
  char params_file[256];
  sprintf(params_file, "%s/%s", opt_info_dir, argv[ARG_PARAM_FILE_IN]);
  readParams(params_file, params);

 // if (params[7] > 10.0)
 //   params[7] /= GRAV_FACTOR;

  // CONSTANT_THREAD_CURVATURE_ERROR
  BEND_COEFF = params[0];
  TWIST_COEFF = params[1];
  GRAV_COEFF = params[2];

  opt_params_thread_params.orig_coeffs.resize(3);
  for (int i=0; i < 3; i++)
  {
    opt_params_thread_params.orig_coeffs[i] = params[i];
  }


  char thread_file[256];
  sprintf(thread_file, "%s/%s", opt_info_dir, argv[ARG_THREAD_FILE]); 
  prepareOptimizationParams(thread_file);


  opt_params_thread_params.orig_params = new NEWMAT::ColumnVector(2);

  opt_params_thread_params.orig_params->element(0) = BEND_COEFF;
  opt_params_thread_params.orig_params->element(1) = TWIST_COEFF;

  char start_params_file[256];
  sprintf(start_params_file, "%sstart.params", data_write_base);
  writeParams(start_params_file, params);


  NEWMAT::ColumnVector x_sol;

  time_t start, end;
  time(&start);



/*  int dim_constraint=5;
  NEWMAT::ColumnVector lower(dim_constraint), upper(dim_constraint);
  lower << 0.0 << 0.0 << 0.0 << 0.0 << 0.0;
  upper << 100.0 << 100.0 << 100.0 << 100.0 << 10000.0;
  OPTPP::Constraint c1 = new OPTPP::BoundConstraint(dim_constraint, lower, upper);
  OPTPP::CompoundConstraint* constraints = new OPTPP::CompoundConstraint(c1);
*/

  OPTPP::NLF0 nlp(2, threadParamEvalFunction, threadParamEvalFunction_init);
  OPTPP::OptPDS objfcn(&nlp);
  objfcn.setFcnTol(1.e-8);
  objfcn.setSSS(256);
  objfcn.setMaxIter(5000);
  objfcn.setMaxFeval(20000);
//  objfcn.setSSS(1);
//  objfcn.setMaxIter(1);
//  objfcn.setMaxFeval(1);
  objfcn.setStepTol(1.e-6);

  char objfcn_out_file[256];
  sprintf(objfcn_out_file, "%sobjfcn_out.optimize.txt", data_write_base);
  objfcn.setOutputFile(objfcn_out_file,0);


  objfcn.optimize();
  objfcn.printStatus("Solution: ");
  x_sol = nlp.getXc();



  time(&end);
  double diff_time = difftime(end,start);
  printf("took %.2lf seconds\n", diff_time);

  std::cout << "final params:" << std::endl;
  std::cout << x_sol(1) << std::endl;
  std::cout << x_sol(2) << std::endl;



  std::cout << "orig: " << std::endl;

  std::cout << opt_params_thread_params.orig_params->element(0) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(1) << std::endl;


  params[1] = x_sol(1);
  params[2] = x_sol(2);

  char endparams_file[256];
  sprintf(endparams_file, "%send.params", data_write_base);
  writeParams(endparams_file, params);



  char intermediate_params_file[256];
  sprintf(intermediate_params_file, "%sintermediate_data", data_write_base);
  ofstream intermediate_out(intermediate_params_file);
  int dim = 2;
  for (int i = 0; i < opt_params_thread_params.savedParams.size(); i++)
  {
    intermediate_out << "iter: " << opt_params_thread_params.savedParams[i](dim+2) << "  val: " << opt_params_thread_params.savedParams[i](dim+1) << "  ";
    intermediate_out << "params: ";
    for (int j=1; j <= dim; j++)
    {
      intermediate_out << opt_params_thread_params.savedParams[i](j) << "  ";
    }
    intermediate_out << "\n";
  }
  char time_for_file[256];
  sprintf(time_for_file, "took %.2lf seconds", diff_time);
  intermediate_out << time_for_file;
  intermediate_out.close();

/*

  //print the per-trajectory data
  char per_sample_scores_file[256];
  sprintf(per_sample_scores_file, "%s/perSamplesScore", opt_info_dir);
  ofstream per_samples_out(per_sample_scores_file);
  for (int trajNum=0; trajNum < opt_params_thread_params.scoresPerThread.size(); trajNum++)
  {
    for (int threadNum = 0 ; threadNum < opt_params_thread_params.scoresPerThread[trajNum].size(); threadNum++)
    {
      per_samples_out << trajNum << "  " << threadNum << "  " << (opt_params_thread_params.scoresPerThread[trajNum][threadNum]/opt_params_thread_params.f_eval_successful_num) << "\n";
    }
  }
  per_samples_out.close();

*/

/*
  char end_test_file[256];
  sprintf(end_test_file, "%s/test/", opt_info_dir);
  char num_test_file[256];
  sprintf(num_test_file, "%s/numtest", opt_info_dir);
  prepareOptimizationParams(end_test_file, num_test_file);
  double fx;
  int result;
  threadParamEvalFunction(5, x_sol, fx, result);
  char end_test_score_file[256];
  sprintf(end_test_score_file, "%s/testscore", opt_info_dir);
  ofstream out(end_test_score_file);
  out << fx << endl;
  out.close();
  */

  std::cout << "finished " << std::endl;
}


void threadParamEvalFunction_init(int ndim, NEWMAT::ColumnVector& x)
{
	for (int i = 0; i < ndim; i++)
	{
		x.element(i) = opt_params_thread_params.orig_params->element(i);
	}
}

void threadParamEvalFunction(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result)
{

  vector<double> scores_each_thread(opt_params_thread_params.all_threads.size());
  #pragma omp parallel for num_threads(NUM_PARALLEL)
  for (int i=0; i < opt_params_thread_params.all_threads.size(); i++)
  {
    vector<Vector3d> points;
    vector<double> twist_angles;
    Thread toProcess(opt_params_thread_params.all_threads[i]);	//must make a copy!
    
    toProcess.set_coeffs_normalized(x(1), x(2), opt_params_thread_params.orig_coeffs[2]);

		toProcess.minimize_energy();
		toProcess.get_thread_data(points, twist_angles);
		scores_each_thread[i] = 0.0;
		for (int j=0; j < points.size(); j++)
		{
			scores_each_thread[i] += pow((points[j] - opt_params_thread_params.orig_points[i][j]).norm(),2);
		}
		scores_each_thread[i] = sqrt(scores_each_thread[i]);
		//std::cout << scores_each_thread[i] << std::endl;
	}

  fx = 0.0;
	for (int i=0; i < opt_params_thread_params.all_threads.size(); i++)
	{
		fx += scores_each_thread[i];
	}

	fx /= (double)scores_each_thread.size();
  opt_params_thread_params.f_eval_num++;

  std::cout << "" << x(1) << "  " << x(2) << " with score: " << fx <<  std::endl;


  if (fx < opt_params_thread_params.bestVal)
  {
    NEWMAT::ColumnVector toAdd(ndim+2);
    for (int i = 1; i <= ndim; i++)
    {
      toAdd(i) = x(i);
    }
    toAdd(ndim+1) = fx;
    toAdd(ndim+2) = opt_params_thread_params.f_eval_num;
    opt_params_thread_params.savedParams.push_back(toAdd);
    opt_params_thread_params.bestVal = fx;
    std::cout << "best: " << opt_params_thread_params.bestVal << "  " << opt_params_thread_params.f_eval_num << std::endl;
  }

}



/*
void calculatePoints()
{
  NEWMAT::ColumnVector initX(5);
  initX(1) = 0.537;
  initX(2) = 0.03448;
  initX(3) = 1.457;
  initX(4) = 1.502;
  initX(5) = 12.58;

  ofstream pointfile;
  pointfile.precision(10);
  pointfile.open("energy_eval_points.txt");

  double noise = 0.01;
  for (int i=0; i < 70; i++)
  {
    NEWMAT::ColumnVector xcurr(5);
    xcurr.element(0) = randomMaxAbsValue(initX(1)*noise)+initX(1);
    xcurr.element(1) = randomMaxAbsValue(initX(2)*noise)+initX(2);
    xcurr.element(2) = randomMaxAbsValue(initX(3)*noise)+initX(3);
    xcurr.element(3) = randomMaxAbsValue(initX(4)*noise)+initX(4);
    xcurr.element(4) = randomMaxAbsValue(initX(5)*noise)+initX(5);

    double valcurr;
    int trash;
    threadParamEvalFunction(5, xcurr, valcurr, trash);

    pointfile << xcurr(1) << "  " << xcurr(2) << "  " << xcurr(3) << "  " << xcurr(4) << "  " << xcurr(5) << "   " << valcurr << std::endl;
  }

  pointfile.close();

}*/


void prepareOptimizationParams(char* thread_file) 
{
  Trajectory_Reader traj_reader(thread_file);
  traj_reader.read_threads_from_file();
  

	opt_params_thread_params.all_threads.resize(0);
  opt_params_thread_params.orig_points.resize(traj_reader.get_all_threads().size());
	opt_params_thread_params.orig_twist_angles.resize(traj_reader.get_all_threads().size());
  for (int i=0; i < traj_reader.get_all_threads().size(); i++)
  {
		opt_params_thread_params.all_threads.push_back(traj_reader.get_all_threads()[i]);
		opt_params_thread_params.all_threads.back().get_thread_data(opt_params_thread_params.orig_points[i], opt_params_thread_params.orig_twist_angles[i]);
  }



  opt_params_thread_params.f_eval_num = 0;
  opt_params_thread_params.bestVal = DBL_MAX;

}


/*
double medianOfVecs(vector<vector<double> >& scores_in)
{
  vector<double> scores;

  for (int i=0; i < scores_in.size(); i++)
  {
    for (int j=0; j < scores_in[i].size(); j++)
    {
      scores.push_back(scores_in[i][j]);
    }
  }

  double median;
  size_t size = scores.size();

  sort(scores.begin(), scores.end());

  if (size  % 2 == 0)
  {
      median = (scores[size / 2 - 1] + scores[size / 2]) / 2;
  }
  else 
  {
      median = scores[size / 2];
  }

  return median;
}*/

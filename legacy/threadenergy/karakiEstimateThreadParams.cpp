#include "estimate_thread_params.h"
#include <time.h>
#include <sys/time.h>

int main (int argc, char * argv[])
{

  if (argc > 2 && !strcmp(argv[1], "est"))
  {
    sprintf(opt_info_dir, "%s", argv[2]);
  } else {
    sprintf(opt_info_dir, "results/latest");
  }


  double* params = new double[8];
  char params_file[256];
  sprintf(params_file, "%s/test.params", opt_info_dir);
  readParams(params_file, params);
  // CONSTANT_THREAD_CURVATURE_ERROR
  double CONSTANT_THREAD_CURVATURE_ERROR_PENALTY = params[2];
  double CONSTANT_THREAD_TORSION_ERROR_PENALTY = params[3];
  double CONSTANT_THREAD_DIFF_CURVATURE_ERROR_PENALTY = 0.0;
  double CONSTANT_THREAD_DIFF_TORSION_ERROR_PENALTY = 0.0;
  double GRAVITY_CONSTANT = 0.0;



 char base_dir_training[256];
  sprintf(base_dir_training, "%s/training/", opt_info_dir);
  char numtraining_file[256];
  sprintf(numtraining_file, "%s/numtraining", opt_info_dir);
  prepareOptimizationParams(base_dir_training, numtraining_file);

  time_t start, end;

  srand(time(NULL));
  srand((unsigned int)time((time_t *)NULL));


  double noise = 0.0;

  opt_params_thread_params.orig_params->element(0) = abs(randomMaxAbsValue(CONSTANT_THREAD_CURVATURE_ERROR_PENALTY*noise)+ CONSTANT_THREAD_CURVATURE_ERROR_PENALTY);
  opt_params_thread_params.orig_params->element(1) = abs(randomMaxAbsValue(CONSTANT_THREAD_TORSION_ERROR_PENALTY*noise)+CONSTANT_THREAD_TORSION_ERROR_PENALTY);

  opt_params_thread_params.bestVal = DBL_MAX;
  opt_params_thread_params.f_eval_num = 0;
  opt_params_thread_params.f_eval_successful_num = 0;

  params[2] = opt_params_thread_params.orig_params->element(0);
  params[3] = opt_params_thread_params.orig_params->element(1);
  params[4] = 0.0;
  params[5] = 0.0;
  params[7] = 0.0;
  char start_params_file[256];
  sprintf(start_params_file, "%s/start.params", opt_info_dir);
  writeParams(start_params_file, params);


  std::cout << opt_params_thread_params.orig_params->element(0) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(1) << std::endl;

  NEWMAT::ColumnVector x_sol;

  time(&start);



  //optimize_FDNLF(8, threadParamEvalFunction, threadParamEvalFunction_init, x_sol);
/*
  OPTPP::FDNLF1 nlp(5, threadParamEvalFunction, threadParamEvalFunction_init);

  OPTPP::OptQNewton objfcn(&nlp);

  objfcn.setSearchStrategy(OPTPP::TrustRegion);
  objfcn.setMaxFeval(20000);
  objfcn.setMaxIter(5000);
  objfcn.setFcnTol(1.e-7);
  //objfcn.setMaxStep(1.0e-1);
  //
 objfcn.setOutputFile("PARAM_EST_OUT_DERIV_NEAR.txt",0);


/*
  OPTPP::NLF0 nlp(5, threadParamEvalFunction, threadParamEvalFunction_init);
  OPTPP::GenSetStd gs(5);
  OPTPP::OptGSS objfcn(&nlp, &gs);
  objfcn.setMaxIter(5000);
  objfcn.setMaxFeval(50000);
  objfcn.setFcnTol(1.e-7);
  objfcn.setFullSearch(true);
  objfcn.optimize();
 objfcn.setOutputFile("PARAM_EST_OUT_GSS_FAR.txt",0);
*/


  OPTPP::NLF0 nlp(2, threadParamEvalFunction, threadParamEvalFunction_init);
  OPTPP::OptPDS objfcn(&nlp);
  objfcn.setFcnTol(1.e-1);
  objfcn.setSSS(20);
  objfcn.setMaxIter(500);
  objfcn.setMaxFeval(2000);
  objfcn.setStepTol(1.e-5);

  char objfcn_out_file[256];
  sprintf(objfcn_out_file, "%s/objfcn_out.optimize.txt", opt_info_dir);
  objfcn.setOutputFile(objfcn_out_file,0);



  objfcn.optimize();
  objfcn.printStatus("Solution: ");
  x_sol = nlp.getXc();



  time(&end);
  double diff_time = difftime(end,start);
  printf("took %.2lf seconds", diff_time);

  std::cout << x_sol(1) << std::endl;
  std::cout << x_sol(2) << std::endl;

  std::cout << "orig: " << std::endl;

  std::cout << opt_params_thread_params.orig_params->element(0) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(1) << std::endl;


  params[2] = x_sol(1);
  params[3] = x_sol(2);
  params[4] = 0.0;
  params[5] = 0.0;
  params[7] = 0.0;
  char endparams_file[256];
  sprintf(endparams_file, "%s/end.params", opt_info_dir);
  writeParams(endparams_file, params);


  char intermediate_params_file[256];
  sprintf(intermediate_params_file, "%s/intermediate_data", opt_info_dir);
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
  std::cout << opt_params_thread_params.savedParams.size();
  char time_for_file[256];
  sprintf(time_for_file, "took %.2lf seconds", diff_time);
  intermediate_out << time_for_file;
  intermediate_out.close();


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
  
  


  char end_test_file[256];
  sprintf(end_test_file, "%s/test/", opt_info_dir);
  char num_test_file[256];
  sprintf(num_test_file, "%s/numtest", opt_info_dir);
  prepareOptimizationParams(end_test_file, num_test_file);
  double fx;
  int result;
  threadParamEvalFunction(2, x_sol, fx, result);
  char end_test_score_file[256];
  sprintf(end_test_score_file, "%s/testscore", opt_info_dir);
  ofstream out(end_test_score_file);
  out << fx << endl;
  out.close();

  std::cout << "finished " << opt_info_dir << std::endl;

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

  //Thread* currThread = new Thread(opt_params_thread_params.start_thread);
  // currThread->thread_curvature_error_penalty = x(1);
  // currThread->thread_torsion_error_penalty = x(2);
  // currThread->thread_diff_curvature_error_penalty = x(3);
  // currThread->thread_diff_torsion_error_penalty = x(4);
  // currThread->gravity_penalty = x(5)/1000000.0;
  //currThread->position_error_penalty = x(6);
  //currThread->rotation_error_penalty = x(7);
  //currThread->total_error_penalty = x(8)*1000.0;

  fx = 0.0;
  int numThreadsTooSlow = 0;
  int numThreadsSeen = 0;
  bool skipRest = false;
  double last_val = 0;

  vector<vector<double> > these_score;
  for(int n = 0; n < opt_params_thread_params.NUM_TRAJ && !skipRest; n++)
  {
    vector<double> scores_this_traj;
    for (int i=0; i < opt_params_thread_params.savedThreads[n]->size() && !skipRest; i++) //i+= opt_params_thread_params.num_motions_to_sim)
    {
      Thread* currThread = new Thread((*opt_params_thread_params.threads[n])[i]);
      currThread->thread_curvature_error_penalty = x(1);
      currThread->thread_torsion_error_penalty = x(2);
      currThread->thread_diff_curvature_error_penalty = 0.0;
      currThread->thread_diff_torsion_error_penalty = 0.0;
      currThread->gravity_penalty = 0.0;

      //currThread->optimizeManyPoints_startAndEnd_MyParams(opt_params_thread_params.savedThreads[i].points, 2, opt_params_thread_params.savedThreads[i].positions, opt_params_thread_params.savedThreads[i].tangents);

      //   currThread->upsampleAndOptimize_minLength(0.126);
      struct timeval start, end;
      gettimeofday(&start, 0x0); 
      
      currThread->minimize_energy_fixedPieces();

      gettimeofday(&end, 0x0);
      end.tv_usec += 1000000*(end.tv_sec - start.tv_sec);
      double time_elapsed = (end.tv_usec - start.tv_usec)/1000;
      std::cout << "Elapsed time: " << time_elapsed << " milliseconds" << std::endl;


      MatrixXd threadPoints(opt_params_thread_params.num_pts_per[n], 3);
      currThread->getPoints(threadPoints);
      fx += avgDistBetweenPoints(threadPoints, (*opt_params_thread_params.savedThreads[n])[i].points);

      if (time_elapsed > 1500.0)
      {
        numThreadsTooSlow++;
        if (numThreadsTooSlow > 3)
        {
          //time to skip
          fx *= 1.5*(((double)opt_params_thread_params.num_threads_total) / ((double)numThreadsSeen));
          skipRest = true;
        }
      }


      /*Thread* toApplyMotion = new Thread(currThread);
      //std::cout << "ind: " << i << std::endl;
      for (int j = i; j < min(i+opt_params_thread_params.num_motions_to_sim, opt_params_thread_params.motions[n]->size()); j++)
      {
        (*opt_params_thread_params.motions[n])[j].applyMotion_NoCopy(toApplyMotion);
        toApplyMotion->getPoints(threadPoints);
        fx += avgDistBetweenPoints(threadPoints, (*opt_params_thread_params.savedThreads[n])[j+1].points);



        /*     std::cout << "this dist: " << avgDistBetweenPoints(threadPoints, opt_params_thread_params.points[j+1]) << std::endl;
               std::cout << "points orig     points new      norm\n";
               for (int i=0; i < threadPoints.rows(); i++)
               {
               std::cout << (opt_params_thread_params.points[j+1].block(i,0,1,3)) << "               " << (threadPoints.block(i,0,1,3)) << "      " << (opt_params_thread_params.points[j+1].block(i,0,1,3) - threadPoints.block(i,0,1,3)).norm() << std::endl;
               }*/
      //}
      //delete toApplyMotion;

      double this_val = fx - last_val;
      std::cout << "val: " << this_val << std::endl;
      last_val = fx;


      if (fx > 100000)
      {
        fx *= (((double)opt_params_thread_params.num_threads_total )/ ((double)numThreadsSeen));
        skipRest = true;
      }
      
      scores_this_traj.push_back(this_val);

      delete currThread;
    }
    these_score.push_back(scores_this_traj);
  }
  fx /= (double)opt_params_thread_params.num_threads_total;
  opt_params_thread_params.f_eval_num++;


  if (fx < opt_params_thread_params.bestVal)
  {
    NEWMAT::ColumnVector toAdd(ndim+2);
    for (int i = 1; i <= ndim; i++)
    {
      toAdd(i) = x(i);
    }
    toAdd(ndim+1) = fx;
    toAdd(ndim+2) = (double)opt_params_thread_params.f_eval_num;
    opt_params_thread_params.savedParams.push_back(toAdd);
    opt_params_thread_params.bestVal = fx;
  }


 //add to list of scores if necessary
  if (!skipRest)
  {
    opt_params_thread_params.f_eval_successful_num++;
    for (int i = 0; i < these_score.size(); i++)
    {
      for (int j = 0; j < these_score[i].size(); j++)
      {
        opt_params_thread_params.scoresPerThread[i][j] += these_score[i][j];
      }
    }
  }

  fx = medianOfVecs(these_score);
  if (skipRest)
    fx *= 4.0;

  //std::cout << "val: " << fx << std::endl;

  result = OPTPP::NLPFunction;
  std::cout << "val: " << fx << std::endl;
}


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



}

void prepareOptimizationParams(char * basedir, char * numtraining_file) {
  // read in the number of trajectories, create an array of traj readers? yes
  ifstream in(numtraining_file);
  int N;
  if(!in) {
    cout << "error fine not prepared" << endl;
    exit(1);
  }

  in >> N;
  in.close();
  cout << N << endl;

  char motions_name[256];
  char thread_name[256];
  char points_name[256];
  
  opt_params_thread_params.num_threads_total = 0;
  opt_params_thread_params.scoresPerThread.resize(N);
  for(int i = 0; i < N; i++) {
    sprintf(motions_name, "%s/%d_traj.txt", basedir, i);
    sprintf(thread_name, "%s/%d_traj_imgs.txt", basedir, i);
    sprintf(points_name, "%s/%d_traj_imgs.points", basedir, i);
    cout << "Reading in trajectories: " << endl;
    Trajectory_Reader* traj_reader = new Trajectory_Reader(motions_name, thread_name, points_name);
    traj_reader->read_motions_from_file();
    traj_reader->read_threads_from_file();

    cout << "copying pointers" << endl;
    opt_params_thread_params.savedThreads.push_back(traj_reader->get_all_points());
    opt_params_thread_params.threads.push_back(traj_reader->get_all_threads());
    opt_params_thread_params.motions.push_back(traj_reader->get_all_motions());
    opt_params_thread_params.num_pts_per.push_back(opt_params_thread_params.savedThreads[opt_params_thread_params.savedThreads.size()-1]->front().points.rows());
    opt_params_thread_params.length_thread.push_back(traj_reader->length());

    cout << "initializing thread" << endl;
    traj_reader->estimate_init_thread();
    opt_params_thread_params.start_thread.push_back(new Thread(traj_reader->start_thread()));
    
    opt_params_thread_params.num_threads_total += opt_params_thread_params.threads.back()->size();

    //initialize for score tracking per thread
    opt_params_thread_params.scoresPerThread[i].resize(opt_params_thread_params.threads.back()->size(),0.0);
  }
  opt_params_thread_params.num_motions_to_sim = NUM_MOTIONS_TO_SIM;
  opt_params_thread_params.orig_params = new NEWMAT::ColumnVector(5);
  opt_params_thread_params.NUM_TRAJ = N;

}


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
}

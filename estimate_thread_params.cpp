#include "estimate_thread_params.h"
#include <time.h>

int main (int argc, char * argv[]) 
{
  Trajectory_Reader traj_reader;
  traj_reader.read_motions_from_file();
  traj_reader.read_threads_from_file();
  
  opt_params_thread_params.motions = traj_reader.get_all_motions();
  opt_params_thread_params.points = traj_reader.get_all_points();
  opt_params_thread_params.num_pts_per= opt_params_thread_params.points.front().rows();
  opt_params_thread_params.length_thread = traj_reader.length();

  traj_reader.estimate_init_thread();

  opt_params_thread_params.start_thread = new Thread(traj_reader.start_thread());

  opt_params_thread_params.orig_params = new NEWMAT::ColumnVector(5);

  time_t start, end;

  srand(time(NULL));
  srand((unsigned int)time((time_t *)NULL));


  double noise = 0.00;

  opt_params_thread_params.orig_params->element(0) = randomMaxAbsValue(CONSTANT_THREAD_CURVATURE_ERROR_PENALTY*noise)+ CONSTANT_THREAD_CURVATURE_ERROR_PENALTY;
  opt_params_thread_params.orig_params->element(1) = randomMaxAbsValue(CONSTANT_THREAD_TORSION_ERROR_PENALTY*noise)+CONSTANT_THREAD_TORSION_ERROR_PENALTY;
  opt_params_thread_params.orig_params->element(2) = randomMaxAbsValue(CONSTANT_THREAD_DIFF_CURVATURE_ERROR_PENALTY*noise)+CONSTANT_THREAD_DIFF_CURVATURE_ERROR_PENALTY;
  opt_params_thread_params.orig_params->element(3) = randomMaxAbsValue(CONSTANT_THREAD_DIFF_TORSION_ERROR_PENALTY*noise)+CONSTANT_THREAD_DIFF_TORSION_ERROR_PENALTY;
  opt_params_thread_params.orig_params->element(4) = (randomMaxAbsValue(GRAVITY_CONSTANT*noise)+GRAVITY_CONSTANT)*1000000.0;
  opt_params_thread_params.orig_params->element(5) = randomMaxAbsValue(CONSTANT_POSITION_ERROR_PENALTY*noise)+CONSTANT_POSITION_ERROR_PENALTY;
  opt_params_thread_params.orig_params->element(6) = randomMaxAbsValue(CONSTANT_ROTATION_ERROR_PENALTY*noise)+CONSTANT_ROTATION_ERROR_PENALTY;
  opt_params_thread_params.orig_params->element(7) = (randomMaxAbsValue(CONSTANT_TOTAL_ERROR_PENALTY*noise)+CONSTANT_TOTAL_ERROR_PENALTY)/1000.0;


  std::cout << opt_params_thread_params.orig_params->element(0) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(1) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(2) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(3) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(4) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(5) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(6) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(7) << std::endl;

  NEWMAT::ColumnVector x_sol;

  time(&start);



  optimize_FDNLF(8, threadParamEvalFunction, threadParamEvalFunction_init, x_sol);
  /*OPTPP::FDNLF1 nlp(8, threadParamEvalFunction, threadParamEvalFunction_init);

  OPTPP::OptQNewton objfcn(&nlp);

  objfcn.setSearchStrategy(OPTPP::TrustRegion);
  objfcn.setMaxFeval(20000);
  objfcn.setMaxIter(5000);
  objfcn.setFcnTol(1.e-8);
  objfcn.setMaxStep(1.0e-1);

  objfcn.optimize();
  x_sol = nlp.getXc();

*/

  time(&end);
  double diff_time = difftime(end,start);
  printf("took %.2lf seconds", diff_time);

  std::cout << x_sol(1) << std::endl;
  std::cout << x_sol(2) << std::endl;
  std::cout << x_sol(3) << std::endl;
  std::cout << x_sol(4) << std::endl;
  std::cout << (x_sol(5))/1000000.0 << std::endl;
  std::cout << x_sol(6) << std::endl;
  std::cout << x_sol(7) << std::endl;
  std::cout << (x_sol(8))*1000.0 << std::endl;

  std::cout << "orig: " << std::endl;

  std::cout << opt_params_thread_params.orig_params->element(0) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(1) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(2) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(3) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(4) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(5) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(6) << std::endl;
  std::cout << opt_params_thread_params.orig_params->element(7) << std::endl;




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
 /* 
  Thread* currThread = new Thread(opt_params_thread_params.start_thread);

  currThread->thread_curvature_error_penalty = x(1);
  currThread->thread_torsion_error_penalty = x(2);
  currThread->thread_diff_curvature_error_penalty = x(3);
  currThread->thread_diff_torsion_error_penalty = x(4);
  currThread->position_error_penalty = x(5);
  currThread->rotation_error_penalty = x(6);
  currThread->total_error_penalty = x(7)*1000.0;
  currThread->gravity_penalty = x(8)/1000000.0;

  std::cout << x(1) << std::endl;
  std::cout << x(2) << std::endl;
  std::cout << x(3) << std::endl;
  std::cout << x(4) << std::endl;
  std::cout << x(5) << std::endl;
  std::cout << x(6) << std::endl;
  std::cout << (x(7)*1000.0) << std::endl;
  std::cout << (x(8)/1000000.0) << std::endl;

  //we assume we have n sets of points, and n-1 motions, due to having points for the 1st piece
  MatrixXd currPoints(opt_params_thread_params.num_pts_per,3);
  currThread->getPoints(currPoints);
  fx = avgDistBetweenPoints(currPoints, opt_params_thread_params.points.front());
  for (int currInd = 0; currInd < opt_params_thread_params.motions.size(); currInd++)
  {
    Thread* next = opt_params_thread_params.motions[currInd].applyMotion(currThread);
    next->getPoints(currPoints);
    fx += avgDistBetweenPoints(currPoints, opt_params_thread_params.points[currInd+1]);
    delete currThread;
    currThread = next;
  }

  delete currThread;

*/

/*

  fx = 0.0;
  for (int i=0; i < opt_params_thread_params.motions.size(); i++)
  {
    Thread* currThread = new Thread(opt_params_thread_params.points[i], 2, opt_params_thread_params.length_thread);
    currThread->thread_curvature_error_penalty = x(1);
    currThread->thread_torsion_error_penalty = x(2);
    currThread->thread_diff_curvature_error_penalty = x(3);
    currThread->thread_diff_torsion_error_penalty = x(4);
    currThread->position_error_penalty = x(5);
    currThread->rotation_error_penalty = x(6);
    currThread->total_error_penalty = x(7)*1000.0;
    currThread->gravity_penalty = x(8)/1000000.0;
    Thread* afterMotion = opt_params_thread_params.motions[i].applyMotion(currThread);
    MatrixXd afterMotionPoints(opt_params_thread_params.num_pts_per, 3);
    afterMotion->getPoints(afterMotionPoints);
    fx += avgDistBetweenPoints(afterMotionPoints, opt_params_thread_params.points[i+1]);

    delete currThread;
    delete afterMotion;
  }

  std::cout << "val: " << fx << std::endl;

  result = OPTPP::NLPFunction;



  std::cout << x(1) << std::endl;
  std::cout << x(2) << std::endl;
  std::cout << x(3) << std::endl;
  std::cout << x(4) << std::endl;
  std::cout << x(5) << std::endl;
  std::cout << x(6) << std::endl;
  std::cout << x(7) << std::endl;
  std::cout << x(8) << std::endl;


  */

  Thread* currThread = new Thread(opt_params_thread_params.start_thread);
  currThread->thread_curvature_error_penalty = x(1);
  currThread->thread_torsion_error_penalty = x(2);
  currThread->thread_diff_curvature_error_penalty = x(3);
  currThread->thread_diff_torsion_error_penalty = x(4);
  currThread->gravity_penalty = x(5)/1000000.0;
  //currThread->position_error_penalty = x(6);
  //currThread->rotation_error_penalty = x(7);
  //currThread->total_error_penalty = x(8)*1000.0;

  fx = 0.0;
  for (int i=0; i < opt_params_thread_params.motions.size(); i++)
  {
    Thread* afterMotion = opt_params_thread_params.motions[i].applyMotion(currThread);
    MatrixXd afterMotionPoints(opt_params_thread_params.num_pts_per, 3);
    afterMotion->getPoints(afterMotionPoints);
    fx += avgDistBetweenPoints(afterMotionPoints, opt_params_thread_params.points[i+1]);
    delete currThread;
    currThread = new Thread(afterMotion, 16, 2);
    delete afterMotion;
  }

  std::cout << "val: " << fx << std::endl;

  result = OPTPP::NLPFunction;



  std::cout << x(1) << std::endl;
  std::cout << x(2) << std::endl;
  std::cout << x(3) << std::endl;
  std::cout << x(4) << std::endl;
  std::cout << x(5) << std::endl;
  std::cout << x(6) << std::endl;
  std::cout << x(7) << std::endl;
  std::cout << x(8) << std::endl;



}

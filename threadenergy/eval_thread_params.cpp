#include "estimate_thread_params.h"

int main (int argc, char * argv[])
{
  if (argc < 1)
  {
    std::cerr<< "please supply a base directory" << std::endl;
    exit(1);
  }
  std::cout << argv << std::endl;
  exit(0);
/*
  double* params = new double[8];
  readParams("results/latest/test.params", params);

  // CONSTANT_THREAD_CURVATURE_ERROR
  double CONSTANT_THREAD_CURVATURE_ERROR_PENALTY = params[2];
  double CONSTANT_THREAD_TORSION_ERROR_PENALTY = params[3];
  double CONSTANT_THREAD_DIFF_CURVATURE_ERROR_PENALTY = params[4];
  double CONSTANT_THREAD_DIFF_TORSION_ERROR_PENALTY = params[5];
  double GRAVITY_CONSTANT = params[7];



  prepareOptimizationParams("results/latest/training/");

  time_t start, end;

  srand(time(NULL));
  srand((unsigned int)time((time_t *)NULL));


  ifstream in("results/latest/numtraining");
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

  }
  opt_params_thread_params.num_motions_to_sim = NUM_MOTIONS_TO_SIM;
  opt_params_thread_params.orig_params = new NEWMAT::ColumnVector(5);
  opt_params_thread_params.NUM_TRAJ = N;

*/


}


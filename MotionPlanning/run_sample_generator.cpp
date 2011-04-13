#include <boost/program_options.hpp>
#include <boost/timer.hpp>
#include <boost/progress.hpp>
#include "../DiscreteRods/glThread.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <math.h>
#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/trajectory_reader.h"
#include "../DiscreteRods/trajectory_recorder.h" 
#include "planner_utils.h"
#include "linearization_utils.h"
#include <omp.h>
#include "global_filenames.h"
#include "../DiscreteRods/glThread.h"


bool hasnan(Thread* tocheck);

namespace po = boost::program_options;
int main(int argc, char* argv[]) {

  unsigned int num_pairs;
  
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message.")
    ("num,n", po::value<unsigned int>(&num_pairs)->default_value(500), "Number of thread pairs to generate")
    ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    cout << desc; 
    exit(0); 
  }

  Thread_RRT planner;
  Thread* reference = (new GLThread())->getThread();
  reference->minimize_energy();

  int num_links = reference->num_pieces(); 
  vector<Thread*> start_samples(num_pairs);
  vector<Thread*> end_samples(num_pairs); 

  boost::progress_display progress(num_pairs);

  #pragma omp parallel for num_threads(NUM_CPU_THREADS)
  for (int i = 0; i < num_pairs; i++) { 
    Thread* start_sample;
    do {
      start_sample = planner.generateSample(reference);
      start_sample->minimize_energy(20000, 1e-10, 0.2, 1e-11);
    } while (hasnan(start_sample));
    start_samples[i] = start_sample;
    Thread* end_sample;
    do {
      end_sample = planner.generateSample(reference);
      end_sample->minimize_energy(20000, 1e-10, 0.2, 1e-11);
    } while (hasnan(end_sample));
    end_samples[i] = end_sample;
    ++progress;
  }

  char start_filename[1024];
  char end_filename[1024];

  sprintf(start_filename, "%s/%s_%d", BASEFOLDER_INITDATA, BASENAME_STARTTHREADS, num_links);
  sprintf(end_filename, "%s/%s_%d", BASEFOLDER_INITDATA, BASENAME_GOALTHREADS, num_links);


  Trajectory_Recorder recorderS(start_filename);
  Trajectory_Recorder recorderE(end_filename); 

  recorderS.add_threads_to_list(start_samples);
  recorderE.add_threads_to_list(end_samples); 

  recorderS.write_threads_to_file();
  recorderE.write_threads_to_file();

  exit(0);

}


bool hasnan(Thread* tocheck)
{
  vector<Vector3d> points;
  vector<double> twist_angles;
  tocheck->get_thread_data(points, twist_angles);

  //write each point for each thread
  for (int i=0; i < points.size(); i++)
  {
    if (isnan(points[i].norm()))
      return true;
  }

  return false;
}

#include <boost/program_options.hpp>
#include <boost/timer.hpp>
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


#define GOAL_THREAD_FILE "rrt_goal_thread_data" 
#define HISTOGRAM_FILE "benchmark_data/histogram_1.txt"

using namespace std;
using namespace lshkit;
namespace po = boost::program_options;

void benchmark(unsigned int seed, float threshold) {
  // read thread from file
  cout << "Initialize benchmark with (seed="  << seed
    << ", threshold=" << threshold << ")" << endl;
  srand48(seed); 
  Trajectory_Reader r(GOAL_THREAD_FILE);
  r.read_threads_from_file();
  vector<Thread>& threads = r.get_all_threads();
  if (threads.size() <= 0) {
    cout << "Thread file does not contain a thread" << endl;
    return;
  }

  unsigned int currentTime = (unsigned int)time((time_t *)NULL); 
  ofstream outFile;
  char filename[256];
  sprintf(filename, "benchmark_data/%u_%u.txt", seed, currentTime);
  outFile.open(filename); 
  
  Thread* startThread = (new GLThread())->getThread();
  Thread* goalThread = new Thread(threads.front());
  
  startThread->minimize_energy();
  goalThread->minimize_energy();

  Thread_RRT planner; 
  planner.initialize(startThread, goalThread);

  Thread* goal_thread = new Thread();
  Thread* prev_thread = new Thread();
  Thread* next_thread = new Thread(); 
  double bestScore = DBL_MAX;

  outFile << "seed,score,time,nodes" << endl; 
  boost::timer t; 

  do {
    planner.planStep(*goal_thread, *prev_thread, *next_thread);
    if (planner.distanceBetween(next_thread, goalThread) < bestScore) {
      bestScore = planner.distanceBetween(next_thread, goalThread);

      cout << "Statistics: [Seed, Score, Time, Nodes] "; 
      cout << "[" << seed << ", " << bestScore << ", " << t.elapsed() <<
        ", " << (planner.getTree())->size() << "]" << endl;
      outFile << seed << "," << bestScore << "," << t.elapsed() << "," <<
        planner.getTree()->size() << endl; 
    }

    if (planner.getTree()->size() > 60000) {
      break; 
    }


  } while(bestScore > threshold);

  outFile.close(); 
  ofstream histogramFile;
  histogramFile.open(HISTOGRAM_FILE, ios::app); 
  histogramFile << planner.getTree()->size() << endl; 


}

int main(int argc, char* argv[]) {
  
  unsigned int seed;
  float threshold;
  unsigned int currentTime = (unsigned int)time((time_t *)NULL); 
  
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message.")
    (",s", po::value<unsigned int>(&seed)->default_value(currentTime), "")
    (",t", po::value<float>(&threshold)->default_value(10.0), "")
    ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    cout << desc; 
    exit(0); 
  }

  benchmark(seed, threshold); 
  exit(0);

}





#include <boost/program_options.hpp>
#include <boost/timer.hpp>
#include <iostream>
#include <stdio.h>
#include <omp.h> 
#include <signal.h>
#define PARALLEL_BENCHMARK 1

using namespace std;
namespace po = boost::program_options;

omp_lock_t writelock; 

int main(int argc, char* argv[]) {
 
  omp_init_lock(&writelock);
  unsigned int seed;
  float threshold;
  unsigned int numTests; 
  unsigned int currentTime = (unsigned int)time((time_t *)NULL); 
  
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message.")
    (",s", po::value<unsigned int>(&seed)->default_value(currentTime), "")
    (",t", po::value<float>(&threshold)->default_value(10.0), "")
    (",n", po::value<unsigned int>(&numTests)->default_value(1), "")
    ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc; 
    exit(0); 
  }


  signal(SIGINT, &exit); 
  
  srand(seed); 
//#pragma omp parallel for num_threads(PARALLEL_BENCHMARK)
  for (int i = 0; i < numTests; i++) { 
//   omp_set_lock(&writelock);
    char buffer[256]; 
    unsigned int benchmarkSeed = rand(); 
    sprintf(buffer, "./benchmark_planner -t %f -s %u", 
        threshold, benchmarkSeed);
    cout << buffer << endl;
//   omp_unset_lock(&writelock);

    system(buffer); 

  }


  exit(0);

}


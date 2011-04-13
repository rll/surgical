HOSTNAME=-D$(shell uname -n)

UTILS_DIR = ../../utils/
THREAD_DIR = ../DiscreteRods/
VISION_DIR = ../vision/
LIBS = -L/usr/local/lib/ -lcxcore -lcv -lhighgui -lcvaux -lml -lm  -L/usr/lib/  -lPvAPI -lrt -lglut -lgle -llshkit -lgsl -lgslcblas -lboost_program_options-mt 
INCLUDES = -I/usr/local/include/ -I $(UTILS_DIR) -I/usr/local/include/eigen2/ -I/usr/include/boost/ -I/usr/local/include/lshkit/
#INCLUDES = -I/usr/local/include/ -I/usr/local/include/opencv/ -I/usr/local/include/libPvAPI -I $(UTILS_DIR)
OPT_COMP = -DEIGEN_NO_DEBUG -O3 
# OPT_COMP = 
FOPENMP = -fopenmp
CC = g++ $(OPT_COMP) $(COMPILEFLAG) $(FOPENMP) $(HOSTNAME)
LD = g++ $(OPT_COMP) $(LDFLAG) $(FOPENMP) $(HOSTNAME)
# DEBUG = -ggdb
DEBUG = -ggdb
PROFILER =
ISOTROPIC= -DISOTROPIC
FAKEIMS= -DFAKEIMS

#target : testCanny testCapture example_capture_3cam calibrate_robots test_point_distances PositionsOnRequest PositionsOnRequest2 threadStereo

target: test_linearization test_planner run_planner_libs visualize_trajs

test_linearization: test_linearization.cpp clock.o threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o trajectory_reader.o glThread.o linearization_utils.o trajectory_follower.o iterative_control.o
	$(LD) $(ISOTROPIC) $(INCLUDES) $(DEBUG) -o test_linearization test_linearization.cpp clock.o threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o trajectory_reader.o glThread.o linearization_utils.o trajectory_follower.o iterative_control.o $(LIBS) $(PROFILER)

test_planner: test_planner.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o clock.o glThread.o trajectory_reader.o planner_utils.o linearization_utils.o rrt_utils.o trajectory_follower.o iterative_control.o
	$(LD) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -o test_planner test_planner.cpp threadutils_discrete.o threadpiece_discrete.o  thread_discrete.o trajectory_reader.o trajectory_recorder.o clock.o glThread.o planner_utils.o linearization_utils.o rrt_utils.o trajectory_follower.o iterative_control.o $(LIBS) $(PROFILER)

run_planner_libs: run_planner_libs.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o clock.o glThread.o trajectory_reader.o planner_utils.o linearization_utils.o rrt_utils.o trajectory_follower.o iterative_control.o
	$(LD) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -o run_planner_libs run_planner_libs.cpp threadutils_discrete.o threadpiece_discrete.o  thread_discrete.o trajectory_reader.o trajectory_recorder.o clock.o glThread.o planner_utils.o linearization_utils.o rrt_utils.o trajectory_follower.o iterative_control.o $(LIBS) $(PROFILER)

visualize_trajs: visualize_trajs.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o clock.o glThread.o trajectory_reader.o planner_utils.o linearization_utils.o rrt_utils.o trajectory_follower.o iterative_control.o
	$(LD) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -o visualize_trajs visualize_trajs.cpp threadutils_discrete.o threadpiece_discrete.o  thread_discrete.o trajectory_reader.o trajectory_recorder.o clock.o glThread.o planner_utils.o linearization_utils.o rrt_utils.o trajectory_follower.o iterative_control.o $(LIBS) $(PROFILER)

benchmark_planner: benchmark_planner.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o clock.o glThread.o trajectory_reader.o planner_utils.o linearization_utils.o rrt_utils.o
	$(LD) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -o benchmark_planner benchmark_planner.cpp threadutils_discrete.o threadpiece_discrete.o  thread_discrete.o trajectory_reader.o trajectory_recorder.o clock.o glThread.o planner_utils.o linearization_utils.o rrt_utils.o trajectory_follower.o $(LIBS) $(PROFILER)

run_sample_generator: run_sample_generator.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o clock.o glThread.o trajectory_reader.o planner_utils.o linearization_utils.o rrt_utils.o trajectory_follower.o iterative_control.o
	$(LD) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -o run_sample_generator run_sample_generator.cpp threadutils_discrete.o threadpiece_discrete.o  thread_discrete.o trajectory_reader.o trajectory_recorder.o clock.o glThread.o planner_utils.o linearization_utils.o rrt_utils.o trajectory_follower.o iterative_control.o $(LIBS) $(PROFILER)

benchmark_runner: benchmark_runner.cpp
	$(LD) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -o benchmark_runner benchmark_runner.cpp $(LIBS) $(PROFILER)


################################################
#    Object files                              #
################################################
#Misc
utility.o : $(UTILS_DIR)/utility.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c $(UTILS_DIR)/utility.cpp

clock.o : $(UTILS_DIR)/clock.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c $(UTILS_DIR)/clock.cpp
	
UDPSocket.o: $(SOCKET_DIR)UDPSocket.cpp 
	$(CC) $(INCLUDES) $(DEBUG) -c $(SOCKET_DIR)UDPSocket.cpp


#Discrete Rods
thread_discrete.o: $(THREAD_DIR)thread_discrete.cpp $(THREAD_DIR)thread_discrete.h 
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c $(THREAD_DIR)thread_discrete.cpp $(PROFILER) 

threadpiece_discrete.o: $(THREAD_DIR)threadpiece_discrete.cpp $(THREAD_DIR)threadpiece_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c $(THREAD_DIR)threadpiece_discrete.cpp $(PROFILER)

threadutils_discrete.o: $(THREAD_DIR)threadutils_discrete.cpp $(THREAD_DIR)threadutils_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c $(THREAD_DIR)threadutils_discrete.cpp $(PROFILER)

trajectory_recorder.o: $(THREAD_DIR)trajectory_recorder.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c $(THREAD_DIR)trajectory_recorder.cpp $(PROFILER)

trajectory_reader.o: $(THREAD_DIR)trajectory_reader.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c $(THREAD_DIR)trajectory_reader.cpp $(PROFILER)

glThread.o : $(THREAD_DIR)glThread.cpp $(THREAD_DIR)glThread.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c $(THREAD_DIR)glThread.cpp $(PROFILER)



#Motion Planning
linearization_utils.o: linearization_utils.cpp linearization_utils.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c linearization_utils.cpp $(PROFILER)

rrt_utils.o : rrt_utils.cpp rrt_utils.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c rrt_utils.cpp $(PROFILER)

planner_utils.o : planner_utils.cpp planner_utils.h planner_lib.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c planner_utils.cpp $(PROFILER)

trajectory_follower.o : trajectory_follower.cpp trajectory_follower.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c trajectory_follower.cpp $(PROFILER)

iterative_control.o : iterative_control.cpp iterative_control.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c iterative_control.cpp $(PROFILER)

clean:
	rm *.o *.gch test_linearization test_planner benchmark_runner benchmark_planner run_planner_libs visualize_trajs run_sample_generator


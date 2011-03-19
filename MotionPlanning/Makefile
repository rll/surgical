UTILS_DIR = ../../utils/
THREAD_DIR = ../DiscreteRods/
VISION_DIR = ../vision/
LIBS = -L/usr/local/lib/ -lcxcore -lcv -lhighgui -lcvaux -lml -lm  -L/usr/lib/  -lPvAPI -lrt -lglut -lgle
INCLUDES = -I/usr/local/include/ -I $(UTILS_DIR) -I/usr/local/include/eigen2/
#INCLUDES = -I/usr/local/include/ -I/usr/local/include/opencv/ -I/usr/local/include/libPvAPI -I $(UTILS_DIR)
OPT_COMP = -DEIGEN_NO_DEBUG -O3
# OPT_COMP =
FOPENMP = -fopenmp
CC = g++ $(OPT_COMP) $(COMPILEFLAG)
LD = g++ $(OPT_COMP) $(LDFLAG)
# DEBUG = -ggdb
DEBUG = -ggdb
PROFILER =
ISOTROPIC= -DISOTROPIC
FAKEIMS= -DFAKEIMS

#target : testCanny testCapture example_capture_3cam calibrate_robots test_point_distances PositionsOnRequest PositionsOnRequest2 threadStereo

target: test_linearization

test_linearization: test_linearization.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o trajectory_reader.o glThread.o linearization_utils.o
	$(LD) $(ISOTROPIC) $(INCLUDES) $(DEBUG) -o test_linearization test_linearization.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o trajectory_reader.o glThread.o linearization_utils.o $(LIBS) $(PROFILER)



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


clean:
	rm *.o *.gch test_linearization


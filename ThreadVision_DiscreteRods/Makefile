UTILS_DIR = ../../utils/
MESSAGING_DIR = /home/pabbeel/code/master-slave/slaves/
SOCKET_DIR = /home/pabbeel/code/master-slave/
THREAD_DIR = ../DiscreteRods/
VISION_DIR = ../vision/
LIBS = -L/usr/local/lib/ -lcxcore -lcv -lhighgui -lcvaux -lml -lm  -L/usr/lib/  -lPvAPI -lrt -lglut -lgle
#OPTLIBS = -L/usr/local/lib/ -lopt -lnewmat -L/usr/lib/ -lglut -lgle
#FLIBS =  -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1 -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1/../../../../lib -L/lib/../lib -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1/../../.. -L/usr/lib/x86_64-linux-gnu -lgfortranbegin -lgfortran
INCLUDES = -I/usr/local/include/ -I/usr/local/include/opencv/ -I/usr/local/include/libPvAPI -I $(UTILS_DIR)  -I/usr/include/eigen2
#OPT_DEFINES = -DHAVE_NAMESPACES #-DWITH_MPI -DMPICH_IGNORE_CXX_SEEK
OPT_COMP = -DEIGEN_NO_DEBUG -O3
# OPT_COMP =
FOPENMP = -fopenmp
CC = g++ $(OPT_COMP) $(COMPILEFLAG)
LD = g++ $(OPT_COMP) $(LDFLAG)
# DEBUG = -ggdb
DEBUG = -ggdb -O0
PROFILER =
ISOTROPIC=-DISOTROPIC
FAKEIMS= -DFAKEIMS

#target : testCanny testCapture example_capture_3cam calibrate_robots test_point_distances PositionsOnRequest PositionsOnRequest2 threadStereo

target: threadStereo threadStereo_fake

threadStereo: threadStereo_manyThreads.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o threadutils_discrete.o threadpiece_discrete.o thread_discrete.o threadpiece_vision_discrete.o thread_vision_utils.o threadhypoth_vision_discrete.o thread_vision_discrete.o trajectory_recorder.o trajectory_reader.o glThread.o CannyWriter.o
	$(LD) $(ISOTROPIC) $(INCLUDES) $(DEBUG) -o threadStereo threadStereo_manyThreads.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o threadutils_discrete.o threadpiece_discrete.o thread_discrete.o threadpiece_vision_discrete.o thread_vision_utils.o threadhypoth_vision_discrete.o thread_vision_discrete.o trajectory_recorder.o trajectory_reader.o glThread.o CannyWriter.o $(LIBS) $(PROFILER)

threadStereo_fake: threadStereo_manyThreads.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o threadutils_discrete.o threadpiece_discrete.o thread_discrete.o threadpiece_vision_discrete.o thread_vision_utils.o threadhypoth_vision_discrete.o thread_vision_discrete_fake.o trajectory_recorder.o trajectory_reader.o glThread.o CannyWriter.o
	$(LD) $(ISOTROPIC) $(FAKEIMS) $(INCLUDES) $(DEBUG) -o threadStereo_fake threadStereo_manyThreads.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o threadutils_discrete.o threadpiece_discrete.o thread_discrete.o threadpiece_vision_discrete.o thread_vision_utils.o threadhypoth_vision_discrete.o thread_vision_discrete_fake.o trajectory_recorder.o trajectory_reader.o glThread.o CannyWriter.o $(LIBS) $(PROFILER)

estimate_twists: estimate_twists.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o threadutils_discrete.o threadpiece_discrete.o thread_discrete.o threadpiece_vision_discrete.o thread_vision_utils.o threadhypoth_vision_discrete.o thread_vision_discrete.o trajectory_recorder.o trajectory_reader.o glThread.o
	$(LD) $(ISOTROPIC) $(INCLUDES) $(DEBUG) -o estimate_twists estimate_twists.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o threadutils_discrete.o threadpiece_discrete.o thread_discrete.o threadpiece_vision_discrete.o thread_vision_utils.o threadhypoth_vision_discrete.o thread_vision_discrete.o trajectory_recorder.o trajectory_reader.o glThread.o $(LIBS) $(PROFILER)


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

#Vision
CannyOrient.o : $(VISION_DIR)CannyOrient.cpp $(VISION_DIR)CannyOrient.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(VISION_DIR)CannyOrient.cpp
	
utils2.o : $(VISION_DIR)utils2.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c $(VISION_DIR)utils2.cpp

capture_gige2.o : $(VISION_DIR)capture_gige2.cpp $(VISION_DIR)capture2.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(VISION_DIR)capture_gige2.cpp
	
messaging.o: $(MESSAGING_DIR)messaging.cc
	$(CC) $(INCLUDES) $(DEBUG) -c $(MESSAGING_DIR)messaging.cc

StereoOnClicks.o: $(VISION_DIR)StereoOnClicks.cpp 
	$(CC) $(INCLUDES) $(DEBUG) -c $(VISION_DIR)StereoOnClicks.cpp
	
StereoOnClicks2.o: $(VISION_DIR)StereoOnClicks2.cpp 
	$(CC) $(INCLUDES) $(DEBUG) -c $(VISION_DIR)StereoOnClicks2.cpp	

ThreeCam.o: $(VISION_DIR)ThreeCam.cpp $(VISION_DIR)ThreeCam.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(VISION_DIR)ThreeCam.cpp


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


#Thread Vision
thread_vision_discrete.o: thread_vision_discrete.cpp thread_vision_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c thread_vision_discrete.cpp  $(PROFILER)

thread_vision_discrete_fake.o: thread_vision_discrete.cpp thread_vision_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(FAKEIMS) $(DEBUG) -c thread_vision_discrete.cpp -o thread_vision_discrete_fake.o  $(PROFILER)

threadhypoth_vision_discrete.o: threadhypoth_vision_discrete.cpp threadhypoth_vision_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c threadhypoth_vision_discrete.cpp $(PROFILER)

threadpiece_vision_discrete.o: threadpiece_vision_discrete.cpp threadpiece_vision_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c threadpiece_vision_discrete.cpp $(PROFILER)

thread_vision_utils.o: thread_vision_utils.cpp thread_vision_utils.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c thread_vision_utils.cpp $(PROFILER)

CannyWriter.o: CannyWriter.h CannyWriter.cpp
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c CannyWriter.cpp $(PROFILER)


clean:
	rm *.o *.gch threadStereo threadStereo_fake estimate_twists



MACHINE= $(shell uname -s)

ifeq ($(MACHINE),Darwin)
    COMPILEFLAG=-DMAC
    LDFLAG=$(COMPILEFLAG) -L"/System/Library/Frameworks/OpenGL.framework/Libraries" -L"/System/Library/Frameworks/GLUT.framework" -framework GLUT -framework OpenGL -lGL -lGLU -L/usr/X11R6/lib
    VISION_LIBS=-L/opt/local/lib/
    LIBS=$(VISION_LIBS) -lopencv_core -lopencv_legacy -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_contrib
else
    COMPILEFLAG=
    LDFLAG=$(COMPILEFLAG)
    VISION_LIBS=-L/usr/local/lib/ -L/usr/lib/
    LIBS = $(VISION_LIBS) -lcxcore -lcv -lhighgui -lcvaux -lml -lm  -lPvAPI -lrt -lglut -lgle
endif


UTILS_DIR = ../../../utils/
MESSAGING_DIR = /home/pabbeel/code/master-slave/slaves/
SOCKET_DIR = /home/pabbeel/code/master-slave/
THREAD_DIR = ../
VISION_DIR = ../../vision/
OPTLIBS = -L/usr/local/lib/ -lopt -lnewmat
FLIBS =  -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1 -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1/../../../../lib -L/lib/../lib -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1/../../.. -L/usr/lib/x86_64-linux-gnu -lgfortranbegin -lgfortran
INCLUDES = -I/usr/local/include/ -I/usr/local/include/opencv/ -I/usr/local/include/libPvAPI -I $(UTILS_DIR) -I/opt/local/include -I/opt/local/opencv/include/
OPT_DEFINES = -DHAVE_NAMESPACES #-DWITH_MPI -DMPICH_IGNORE_CXX_SEEK
OPT_COMP = -DEIGEN_NO_DEBUG -O3 -fopenmp
CC = g++  $(OPT_COMP) $(COMPILEFLAG)
LD = g++  $(OPT_COMP) $(LDFLAG)
#DEBUG = -ggdb
DEBUG = 
ISOTROPIC = -DISOTROPIC


#target : testCanny testCapture example_capture_3cam calibrate_robots test_point_distances PositionsOnRequest PositionsOnRequest2 threadStereo

ifeq ($(MACHINE), Darwin)
    target: try_energy_params process_clicked_points_noniso view_processed_points
else
    target: try_energy_params estimate_thread_params try_energy_params_noniso view_processed_points process_clicked_points_noniso
endif

extractData: extractData.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks_withAuto.o
	$(LD) $(INCLUDES) $(DEBUG) -o extractData extractData.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks_withAuto.o $(LIBS) $(PROFILER)

process_clicked_points: process_clicked_points.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o
	$(LD) $(INCLUDES) $(DEBUG) -o process_clicked_points process_clicked_points.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o $(LIBS) $(PROFILER)

view_processed_points: view_processed_points.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_reader.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o
	$(LD) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -o view_processed_points view_processed_points.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_reader.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o $(LIBS) $(PROFILER)

view_processed_points_move: view_processed_points_move.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_reader.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o
	$(LD) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -o view_processed_points_move view_processed_points_move.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_reader.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o $(LIBS) $(PROFILER)

view_processed_points_noniso: view_processed_points.cpp threadutils_discrete_noniso.o threadpiece_discrete_noniso.o thread_discrete_noniso.o trajectory_reader.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o
	$(LD) $(INCLUDES)$(DEBUG) -o view_processed_points_noniso view_processed_points.cpp threadutils_discrete_noniso.o threadpiece_discrete_noniso.o thread_discrete_noniso.o trajectory_reader.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o $(LIBS) $(PROFILER)

process_clicked_points_noniso: process_clicked_points_noniso.cpp threadutils_discrete_noniso.o threadpiece_discrete_noniso.o thread_discrete_noniso.o trajectory_recorder.o
	$(LD) $(INCLUDES) $(DEBUG) -o process_clicked_points_noniso process_clicked_points_noniso.cpp threadutils_discrete_noniso.o threadpiece_discrete_noniso.o thread_discrete_noniso.o trajectory_recorder.o $(LIBS) $(PROFILER)

estimate_thread_params: estimate_thread_params.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_reader.o
	$(LD) $(OPT_DEFINES) $(ISOTROPIC) $(INCLUDES) $(DEBUG) -o estimate_thread_params estimate_thread_params.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_reader.o $(OPTLIBS) $(FLIBS) $(PROFILER)

try_energy_params_move: try_energy_params_move.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_reader.o
	$(LD) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -DISOTROPIC -o try_energy_params_move try_energy_params_move.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_reader.o $(PROFILER)

try_energy_params: try_energy_params.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_reader.o
	$(LD) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -DISOTROPIC -o try_energy_params try_energy_params.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_reader.o $(PROFILER)

try_energy_params_move_noniso: try_energy_params_move_noniso.cpp threadutils_discrete_noniso.o threadpiece_discrete_noniso.o thread_discrete_noniso.o trajectory_reader.o
	$(LD) $(INCLUDES) $(DEBUG) -o try_energy_params_move_noniso try_energy_params_move_noniso.cpp threadutils_discrete_noniso.o threadpiece_discrete_noniso.o thread_discrete_noniso.o trajectory_reader.o $(PROFILER)

try_energy_params_noniso: try_energy_params_noniso.cpp threadutils_discrete_noniso.o threadpiece_discrete_noniso.o thread_discrete_noniso.o trajectory_reader.o
	$(LD) $(INCLUDES) $(DEBUG) -o try_energy_params_noniso try_energy_params_noniso.cpp threadutils_discrete_noniso.o threadpiece_discrete_noniso.o thread_discrete_noniso.o trajectory_reader.o $(PROFILER)
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

StereoOnClicks_withAuto.o: StereoOnClicks_withAuto.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c StereoOnClicks_withAuto.cpp

ThreeCam.o: $(VISION_DIR)ThreeCam.cpp $(VISION_DIR)ThreeCam.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(VISION_DIR)ThreeCam.cpp


#Discrete Rods
thread_discrete.o: $(THREAD_DIR)thread_discrete.cpp $(THREAD_DIR)thread_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c $(THREAD_DIR)thread_discrete.cpp $(PROFILER)

threadpiece_discrete.o: $(THREAD_DIR)threadpiece_discrete.cpp $(THREAD_DIR)threadpiece_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c $(THREAD_DIR)threadpiece_discrete.cpp $(PROFILER)

threadutils_discrete.o: $(THREAD_DIR)threadutils_discrete.cpp $(THREAD_DIR)threadutils_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c $(THREAD_DIR)threadutils_discrete.cpp $(PROFILER)

thread_discrete_noniso.o: $(THREAD_DIR)thread_discrete.cpp $(THREAD_DIR)thread_discrete.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(THREAD_DIR)thread_discrete.cpp -o thread_discrete_noniso.o $(PROFILER)

threadpiece_discrete_noniso.o: $(THREAD_DIR)threadpiece_discrete.cpp $(THREAD_DIR)threadpiece_discrete.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(THREAD_DIR)threadpiece_discrete.cpp -o threadpiece_discrete_noniso.o $(PROFILER)

threadutils_discrete_noniso.o: $(THREAD_DIR)threadutils_discrete.cpp $(THREAD_DIR)threadutils_discrete.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(THREAD_DIR)threadutils_discrete.cpp -o threadutils_discrete_noniso.o $(PROFILER)


trajectory_recorder.o: $(THREAD_DIR)trajectory_recorder.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c $(THREAD_DIR)trajectory_recorder.cpp $(PROFILER)

trajectory_reader.o: $(THREAD_DIR)trajectory_reader.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c $(THREAD_DIR)trajectory_reader.cpp $(PROFILER)


#Thread Vision
thread_vision_discrete.o: thread_vision_discrete.cpp thread_vision_discrete.h
	$(CC) $(INCLUDES) $(DEBUG) -c thread_vision_discrete.cpp  $(PROFILER)

threadpiece_vision_discrete.o: threadpiece_vision_discrete.cpp threadpiece_vision_discrete.h
	$(CC) $(INCLUDES) $(DEBUG) -c threadpiece_vision_discrete.cpp $(PROFILER)


clean:
	rm *.o *.gch extractData process_clicked_points estimate_thread_params view_processed_points view_processed_points_noniso try_energy_params try_energy_params_noniso process_clicked_points_noniso try_energy_params_move try_energy_params_move_noniso view_processed_points_move



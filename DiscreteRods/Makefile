MACHINE= $(shell uname -s)
HOSTNAME=-D$(shell uname -n | sed 's/-/_/g')

ifeq ($(MACHINE),Darwin)
    LIBS = -L/usr/lib/ -lglut -lgle -L/usr/local/lib/x86_64
    COMPILEFLAG=-DMAC
    LDFLAG=$(COMPILEFLAG) -L"/System/Library/Frameworks/OpenGL.framework/Libraries" -L"/System/Library/Frameworks/GLUT.framework" -framework GLUT -framework OpenGL -lGL -lGLU -L/usr/X11R6/lib
else
    LIBS = -L/usr/lib/ -lglut -lgle
    COMPILEFLAG=
    LDFLAG=
endif

VISION_LIBS = -L/usr/local/lib/ -lcxcore -lcv -lhighgui -lcvaux -lml -lm  -L/usr/lib/  -lPvAPI -lrt
UTILS_DIR = ../utils/
INCLUDES = -I/usr/local/include/ -I/usr/include/ -I $(UTILS_DIR) -I/usr/include/eigen2
VISION_INCLUDES = -I/usr/local/include/opencv/
# OPT_COMP =
OPT_COMP = -DEIGEN_NO_DEBUG -O3
FOPENMP = -fopenmp
CC = g++ $(OPT_COMP) $(COMPILEFLAG) $(HOSTNAME)
LD = g++ $(OPT_COMP) $(LDFLAG) $(HOSTNAME) 
DEBUG = -ggdb
#DEBUG =
PROFILER =
ISOTROPIC=-DISOTROPIC

target: testThreadDiscrete 

testThreadDiscrete: testThreadDiscrete.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o
	$(LD) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -o testThreadDiscrete testThreadDiscrete.cpp threadutils_discrete.o threadpiece_discrete.o  thread_discrete.o trajectory_recorder.o $(LIBS) $(PROFILER)

testThreadDiscrete_noniso: testThreadDiscrete.cpp threadutils_discrete_noniso.o threadpiece_discrete_noniso.o thread_discrete_noniso.o
	$(LD) $(INCLUDES) $(DEBUG) -o testThreadDiscrete_noniso testThreadDiscrete.cpp threadutils_discrete_noniso.o threadpiece_discrete_noniso.o thread_discrete_noniso.o $(LIBS) $(PROFILER)

recordThreadVideo: recordThreadVideo.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_reader.o
	$(LD) $(INCLUDES) $(VISION_INCLUDES) $(DEBUG) -o recordThreadVideo recordThreadVideo.cpp threadutils_discrete.o threadpiece_discrete.o  thread_discrete.o trajectory_reader.o $(LIBS) $(VISION_LIBS)

saveThreadTrajectory: saveThreadTrajectory.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o clock.o
	$(LD) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -o saveThreadTrajectory saveThreadTrajectory.cpp threadutils_discrete.o threadpiece_discrete.o  thread_discrete.o trajectory_recorder.o clock.o $(LIBS)

planThreadTrajectory: planThreadTrajectory.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o clock.o glThread.o trajectory_reader.o thread_discrete_RRT.o
	$(LD) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -o planThreadTrajectory planThreadTrajectory.cpp threadutils_discrete.o threadpiece_discrete.o  thread_discrete.o trajectory_reader.o trajectory_recorder.o clock.o glThread.o thread_discrete_RRT.o $(LIBS)

thread_visualize_and_save: thread_visualize_and_save.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o trajectory_reader.o glThread.o
	$(LD) $(ISOTROPIC) $(INCLUDES) $(VISION_INCLUDES) $(DEBUG) -o thread_visualize_and_save thread_visualize_and_save.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o trajectory_reader.o glThread.o $(LIBS) $(VISION_LIBS) $(PROFILER)

visualizeFrames: visualizeFrames.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o trajectory_reader.o glThread_stripe.o text3d.o
	$(LD) $(ISOTROPIC) $(INCLUDES) $(VISION_INCLUDES) $(DEBUG) -o visualizeFrames visualizeFrames.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o trajectory_reader.o glThread_stripe.o text3d.o $(LIBS) $(VISION_LIBS) $(PROFILER)

visualizeEnergyParams: visualizeEnergyParams.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o trajectory_reader.o glThread_stripe.o text3d.o
	$(LD) $(ISOTROPIC) $(INCLUDES) $(VISION_INCLUDES) $(DEBUG) -o visualizeEnergyParams visualizeEnergyParams.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o trajectory_recorder.o trajectory_reader.o glThread_stripe.o text3d.o $(LIBS) $(VISION_LIBS) $(PROFILER)

thread_visualize_and_save_noniso: thread_visualize_and_save.cpp threadutils_discrete.o threadpiece_discrete_noniso.o thread_discrete_noniso.o trajectory_recorder.o trajectory_reader.o glThread_noniso.o
	$(LD) $(INCLUDES) $(VISION_INCLUDES) $(DEBUG) -o thread_visualize_and_save_noniso thread_visualize_and_save.cpp threadutils_discrete.o threadpiece_discrete_noniso.o thread_discrete_noniso.o trajectory_recorder.o trajectory_reader.o glThread_noniso.o $(LIBS) $(VISION_LIBS) $(PROFILER)




################################################
#    Object files                              #
################################################
thread_discrete.o: thread_discrete.cpp thread_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c thread_discrete.cpp $(PROFILER)

threadpiece_discrete.o: threadpiece_discrete.cpp threadpiece_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c threadpiece_discrete.cpp $(PROFILER)

threadutils_discrete.o: threadutils_discrete.cpp threadutils_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c threadutils_discrete.cpp $(PROFILER)


thread_discrete_noniso.o: thread_discrete.cpp thread_discrete.h
	$(CC) $(INCLUDES) $(DEBUG) -c thread_discrete.cpp -o thread_discrete_noniso.o $(PROFILER)

threadpiece_discrete_noniso.o: threadpiece_discrete.cpp threadpiece_discrete.h
	$(CC) $(INCLUDES) $(DEBUG) -c threadpiece_discrete.cpp -o threadpiece_discrete_noniso.o $(PROFILER)

threadutils_discrete_noniso.o: threadutils_discrete.cpp threadutils_discrete.h
	$(CC) $(INCLUDES) $(DEBUG) -c threadutils_discrete.cpp -o threadutils_discrete_noniso.o $(PROFILER)


trajectory_recorder.o: trajectory_recorder.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c trajectory_recorder.cpp $(PROFILER)

trajectory_reader.o: trajectory_reader.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c trajectory_reader.cpp $(PROFILER)

clock.o : $(UTILS_DIR)/clock.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c $(UTILS_DIR)/clock.cpp



glThread.o : glThread.cpp glThread.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c glThread.cpp $(PROFILER)

glThread_noniso.o : glThread.cpp glThread.h
	$(CC) $(INCLUDES) $(DEBUG) -c glThread.cpp -o glThread_noniso.o $(PROFILER)

glThread_stripe.o : glThread_stripe.cpp glThread_stripe.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c glThread_stripe.cpp $(PROFILER)

thread_discrete_RRT.o : thread_discrete_RRT.cpp thread_discrete_RRT.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c thread_discrete_RRT.cpp $(PROFILER)

text3d.o: text3d.cpp text3d.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) -c text3d.cpp $(PROFILER)



clean:
	rm -r *.o *.gch testThreadDiscrete testThreadDiscrete_noniso saveThreadTrajectory recordThreadVideo thread_visualize_and_save thread_visualize_and_save_noniso *.dSYM



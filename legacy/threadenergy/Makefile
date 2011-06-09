LIBS = -L/usr/local/lib/x86_64 -lopt -lnewmat -L/usr/lib/ -lglut -lgle
FLIBS =  -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1 -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1/../../../../lib -L/lib/../lib -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1/../../.. -L/usr/lib/x86_64-linux-gnu -lgfortranbegin -lgfortran
INCLUDES = -I/usr/local/include/
TOADD = -L"/System/Library/Frameworks/OpenGL.framework/Libraries" -L"/System/Library/Frameworks/GLUT.framework" -framework GLUT -framework OpenGL -lGL -lGLU -L/usr/X11R6/lib -L/opt/local/lib/gcc45/gcc/x86_64-apple-darwin10/4.5.1/
COMPILEFLAG=-DMAC
CC = g++ $(COMPILEFLAG)
LD = g++ $(TOADD) $(COMPILEFLAG)
DEBUG =-ggdb


target: threadVisualizer testRRT estimateThreadParams spliceTrajectory karakiParams

spliceTrajectory: spliceTrajectory.cpp thread_minenergy.o threadpiece_minenergy.o thread_utils.o trajectory_recorder.o trajectory_reader.o
	$(LD) -DHAVE_NAMESPACES -DPDS $(INCLUDES) $(DEBUG) -o spliceTrajectory spliceTrajectory.cpp thread_minenergy.o threadpiece_minenergy.o thread_utils.o trajectory_recorder.o trajectory_reader.o $(LIBS) $(FLIBS)

testOpt: testOpt.cpp testOptFuncs.cpp
	$(LD) -DHAVE_NAMESPACES $(INCLUDES) $(DEBUG) -o testOpt testOpt.cpp testOptFuncs.cpp  $(LIBS) $(FLIBS)

testThread: testThread.cpp thread_minenergy.o threadpiece_minenergy.o thread_utils.o
	$(LD) -DHAVE_NAMESPACES $(INCLUDES) $(DEBUG) -o testThread testThread.cpp thread_minenergy.o threadpiece_minenergy.o thread_utils.o $(LIBS) $(FLIBS)

threadVisualizer: thread_visualizer.cpp thread_minenergy.o threadpiece_minenergy.o thread_utils.o trajectory_recorder.o trajectory_reader.o
	$(LD) -DHAVE_NAMESPACES $(INCLUDES) $(DEBUG) -o threadVisualizer thread_visualizer.cpp thread_minenergy.o threadpiece_minenergy.o thread_utils.o trajectory_recorder.o trajectory_reader.o $(LIBS) $(FLIBS)

testRRT: testRRT.cpp thread_RRT.o thread_minenergy.o threadpiece_minenergy.o thread_utils.o trajectory_recorder.o trajectory_reader.o globals_thread_param_estimation.h
	$(LD) -DHAVE_NAMESPACES $(INCLUDES) $(DEBUG) -o testRRT testRRT.cpp thread_RRT.o thread_minenergy.o threadpiece_minenergy.o thread_utils.o trajectory_recorder.o trajectory_reader.o $(LIBS) $(FLIBS)

estimateThreadParams: estimate_thread_params.cpp thread_minenergy.o threadpiece_minenergy.o thread_utils.o trajectory_recorder.o trajectory_reader.o globals_thread_param_estimation.h
	$(LD) -DHAVE_NAMESPACES -DPDS $(INCLUDES) $(DEBUG) -o estimateThreadParams estimate_thread_params.cpp thread_minenergy.o threadpiece_minenergy.o thread_utils.o trajectory_recorder.o trajectory_reader.o $(LIBS) $(FLIBS)

karakiParams: karakiEstimateThreadParams.cpp thread_minenergy.o threadpiece_minenergy.o thread_utils.o trajectory_recorder.o trajectory_reader.o globals_thread_param_estimation.h
	$(LD) -DHAVE_NAMESPACES -DPDS $(INCLUDES) $(DEBUG) -o karakiParams karakiEstimateThreadParams.cpp thread_minenergy.o threadpiece_minenergy.o thread_utils.o trajectory_recorder.o trajectory_reader.o $(LIBS) $(FLIBS)



################################################
#    Object files                              #
################################################
thread_utils.o: thread_utils.cpp
	$(CC) -DHAVE_NAMESPACES $(INCLUDES) $(DEBUG) -c thread_utils.cpp $(LIBS) $(FLIBS)

thread_minenergy.o : thread_minenergy.cpp
	$(CC) -DHAVE_NAMESPACES $(INCLUDES) $(DEBUG) -c thread_minenergy.cpp $(LIBS) $(FLIBS)

threadpiece_minenergy.o : threadpiece_minenergy.cpp
	$(CC) -DHAVE_NAMESPACES $(INCLUDES) $(DEBUG) -c threadpiece_minenergy.cpp $(LIBS) $(FLIBS)

thread_RRT.o : thread_RRT.cpp
	$(CC) -DHAVE_NAMESPACES $(INCLUDES) $(DEBUG) -c thread_RRT.cpp $(LIBS) $(FLIBS)

trajectory_recorder.o : trajectory_recorder.cpp
	$(CC) -DHAVE_NAMESPACES $(INCLUDES) $(DEBUG) -c trajectory_recorder.cpp $(LIBS) $(FLIBS)

trajectory_reader.o : trajectory_reader.cpp
	$(CC) -DHAVE_NAMESPACES $(INCLUDES) $(DEBUG) -c trajectory_reader.cpp $(LIBS) $(FLIBS)

clean:
	rm *.o testOpt testThread threadVisualizer testRRT estimateThreadParams

MACHINE= $(shell uname -s)
HOSTNAME=-D$(shell uname -n | sed 's/-/_/g' | sed 's/\./_/g')
THREAD_DIR = ../DiscreteRods
UTILS_DIR = ../utils
IO_DIR = $(THREAD_DIR)/IO
ENV_DIR = $(THREAD_DIR)/EnvObjects
COLLISIONS_DIR = $(THREAD_DIR)/Collisions
MP_DIR = ../MotionPlanning
RRT_DIR = .

ifeq ($(MACHINE),Darwin)
    LIBS = -L/usr/lib/ -lglut -lgle -lIL -lGLU -L/opt/local/lib/ -lBulletCollision -lBulletDynamics -lLinearMath -lboost_thread-mt

    COMPILEFLAG=-DMAC
    LDFLAG=$(COMPILEFLAG) -L"/System/Library/Frameworks/OpenGL.framework/Libraries" -L"/System/Library/Frameworks/GLUT.framework" -framework GLUT -framework OpenGL -lGL -lGLU -L/usr/X11R6/lib
    INCLUDES = -I/usr/local/include/ -I/usr/include/ -I $(UTILS_DIR)/ -I/opt/local/include/eigen2/ -I/opt/local/include/ -I/opt/local/include/bullet
else
    LIBS = -L/usr/lib/ -lglut -lgle -lIL -lGLU -lBulletDynamics -lBulletCollision -lboost_thread
    INCLUDES = -I/usr/local/include/ -I/usr/include/ -I $(UTILS_DIR)/ -I/usr/include/eigen2 -I/usr/local/include/bullet
    COMPILEFLAG=
    LDFLAG=-lrt
endif

VISION_LIBS = -L/usr/local/lib/ -lcxcore -lcv -lhighgui -lcvaux -lml -lm  -L/usr/lib/  -lrt #-lPvAPI
VISION_INCLUDES = -I/usr/local/include/opencv/
SOCKETS_DIR = ../utils/sockets/
# OPT_COMP =
OPT_COMP = -Os #-DEIGEN_NO_DEBUG -O3 -DNDEBUG
FOPENMP = -fopenmp
PROFILER = #-L/usr/lib/ -lprofiler
CC = g++ $(OPT_COMP) $(COMPILEFLAG) $(HOSTNAME) $(FOPENMP) $(PROFILER)
LD = g++ $(OPT_COMP) $(LDFLAG) $(HOSTNAME) $(FOPENMP) $(PROFILER)
DEBUG = -ggdb
NDEBUG = #-DNDEBUG
ISOTROPIC=-DISOTROPIC

target: simple_rrt

simple_rrt: simple_rrt.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o ThreadConstrained.o drawUtils.o imageloader.o collisionUtils.o Mouse.o Haptic.o World.o CollisionWorld.o WorldManager.o Capsule.o Control.o Cursor.o EndEffector.o InfinitePlane.o TexturedSphere.o Box.o Needle.o thread_socket_interface.o UDPSocket.o Socket.o StateRecorder.o StateReader.o TrajectoryRecorder.o TrajectoryReader.o worldSQP.o clock.o rrt_utils.o RRTNode.o
	$(LD) $(INCLUDES) $(VISION_INCLUDES) $(ISOTROPIC) $(DEBUG) $(NDEBUG) -o simple_rrt simple_rrt.cpp threadutils_discrete.o threadpiece_discrete.o thread_discrete.o ThreadConstrained.o drawUtils.o imageloader.o collisionUtils.o Mouse.o Haptic.o World.o CollisionWorld.o WorldManager.o Capsule.o Control.o Cursor.o EndEffector.o InfinitePlane.o TexturedSphere.o Box.o Needle.o thread_socket_interface.o UDPSocket.o Socket.o StateRecorder.o StateReader.o TrajectoryRecorder.o TrajectoryReader.o worldSQP.o clock.o rrt_utils.o RRTNode.o $(LIBS) $(VISION_LIBS) $(PROFILER)

################################################
#    Object files                              #
################################################
ThreadConstrained.o: $(THREAD_DIR)/ThreadConstrained.cpp $(THREAD_DIR)/ThreadConstrained.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) $(NDEBUG) -c $(THREAD_DIR)/ThreadConstrained.cpp $(PROFILER)

thread_discrete.o: $(THREAD_DIR)/thread_discrete.cpp $(THREAD_DIR)/thread_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) $(NDEBUG) -c $(THREAD_DIR)/thread_discrete.cpp $(PROFILER)

threadpiece_discrete.o: $(THREAD_DIR)/threadpiece_discrete.cpp $(THREAD_DIR)/threadpiece_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) $(NDEBUG) -c $(THREAD_DIR)/threadpiece_discrete.cpp $(PROFILER)

threadutils_discrete.o: $(THREAD_DIR)/threadutils_discrete.cpp $(THREAD_DIR)/threadutils_discrete.h
	$(CC) $(INCLUDES) $(ISOTROPIC) $(DEBUG) $(NDEBUG) -c $(THREAD_DIR)/threadutils_discrete.cpp $(PROFILER)


Mouse.o: $(IO_DIR)/Mouse.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(IO_DIR)/Mouse.cpp $(PROFILER)

Haptic.o: $(IO_DIR)/Haptic.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(IO_DIR)/Haptic.cpp $(PROFILER)
	
Control.o: $(IO_DIR)/Control.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(IO_DIR)/Control.cpp $(PROFILER)


World.o: $(ENV_DIR)/World.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(ENV_DIR)/World.cpp $(PROFILER)

CollisionWorld.o: $(COLLISIONS_DIR)/CollisionWorld.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(COLLISIONS_DIR)/CollisionWorld.cpp $(PROFILER)

WorldManager.o: $(ENV_DIR)/WorldManager.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(ENV_DIR)/WorldManager.cpp $(PROFILER)

Capsule.o: $(ENV_DIR)/Capsule.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(ENV_DIR)/Capsule.cpp $(PROFILER)

Cursor.o: $(ENV_DIR)/Cursor.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(ENV_DIR)/Cursor.cpp $(PROFILER)

EndEffector.o: $(ENV_DIR)/EndEffector.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(ENV_DIR)/EndEffector.cpp $(PROFILER)

InfinitePlane.o: $(ENV_DIR)/InfinitePlane.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(ENV_DIR)/InfinitePlane.cpp $(PROFILER)
	
TexturedSphere.o: $(ENV_DIR)/TexturedSphere.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(ENV_DIR)/TexturedSphere.cpp $(PROFILER)

Box.o: $(ENV_DIR)/Box.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(ENV_DIR)/Box.cpp $(PROFILER)

Needle.o: $(ENV_DIR)/Needle.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(ENV_DIR)/Needle.cpp $(PROFILER)


rrt_utils.o: $(RRT_DIR)/rrt_utils.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(RRT_DIR)/rrt_utils.cpp $(PROFILER)

RRTNode.o: $(RRT_DIR)/RRTNode.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(RRT_DIR)/RRTNode.cpp $(PROFILER)
	
drawUtils.o: $(UTILS_DIR)/drawUtils.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(UTILS_DIR)/drawUtils.cpp $(PROFILER)
	
imageloader.o: $(UTILS_DIR)/imageloader.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(UTILS_DIR)/imageloader.cpp $(PROFILER)
	
collisionUtils.o: $(COLLISIONS_DIR)/collisionUtils.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(COLLISIONS_DIR)/collisionUtils.cpp $(PROFILER)
	
StateRecorder.o: $(THREAD_DIR)/StateRecorder.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(THREAD_DIR)/StateRecorder.cpp $(PROFILER)

StateReader.o: $(THREAD_DIR)/StateReader.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(THREAD_DIR)/StateReader.cpp $(PROFILER)

TrajectoryRecorder.o: $(THREAD_DIR)/TrajectoryRecorder.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(THREAD_DIR)/TrajectoryRecorder.cpp $(PROFILER)

TrajectoryReader.o: $(THREAD_DIR)/TrajectoryReader.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(THREAD_DIR)/TrajectoryReader.cpp $(PROFILER)


clock.o : $(UTILS_DIR)/clock2.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(UTILS_DIR)/clock.cpp


Socket.o : $(SOCKETS_DIR)/Socket.cpp $(SOCKETS_DIR)/Socket.h
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(SOCKETS_DIR)/Socket.cpp
	
UDPSocket.o : $(SOCKETS_DIR)/UDPSocket.cpp $(SOCKETS_DIR)/UDPSocket.h
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(SOCKETS_DIR)/UDPSocket.cpp

thread_socket_interface.o: $(THREAD_DIR)/thread_socket_interface.cpp $(THREAD_DIR)/thread_socket_interface.h
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(THREAD_DIR)/thread_socket_interface.cpp $(PROFILER)

worldSQP.o: $(MP_DIR)/worldSQP.cpp $(MP_DIR)/worldSQP.h
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(MP_DIR)/worldSQP.cpp $(PROFILER)

clean:
	rm -r *.o *.gch simple_rrt

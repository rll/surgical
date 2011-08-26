UTILS_DIR = ../../utils/
MESSAGING_DIR = ../../legacy/master-slave/slaves/
SOCKET_DIR = ../../utils/sockets/
VISION_DIR = ../
THREAD_DIR = ../../legacy/threadenergy/
LIBS = -L/usr/local/lib/ -lcxcore -lcv -lhighgui -lcvaux -lml -lm -L/usr/lib/  -lPvAPI -lrt 
FLIBS =  -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1 -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1/../../../../lib -L/lib/../lib -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1/../../.. -L/usr/lib/x86_64-linux-gnu -lgfortranbegin -lgfortran -L/usr/local/lib/ -lopt -lnewmat
INCLUDES = -I/usr/local/include/ -I/usr/local/include/eigen2/ -I/usr/local/include/opencv/ -I/usr/local/include/libPvAPI -I $(UTILS_DIR)
CC = g++
LD = g++
DEBUG = -ggdb


target: testCheckerboardFinder saveDataForCalib runCalibFromFiles convertCheckerpointsToRobotFrame

testCheckerboardFinder: testCheckerboardFinder.cpp checkerboard_data_vis.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o
	$(LD) $(INCLUDES) $(DEBUG) -o testCheckerboardFinder testCheckerboardFinder.cpp checkerboard_data_vis.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o $(LIBS) 

saveDataForCalib: saveDataForCalib.cpp checkerboard_data_vis.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o UDPSocket.o messaging.o $(MESSAGING_DIR)debugging.cc 
	$(LD) $(INCLUDES) $(DEBUG) -o saveDataForCalib saveDataForCalib.cpp checkerboard_data_vis.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o UDPSocket.o messaging.o $(MESSAGING_DIR)debugging.cc $(LIBS) 

runCalibFromFiles: runCalibFromFiles.cpp checkerboard_data_vis.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o thread_utils.o
	$(LD) -DHAVE_NAMESPACES $(INCLUDES) $(DEBUG) -o runCalibFromFiles runCalibFromFiles.cpp checkerboard_data_vis.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o thread_utils.o $(LIBS) $(FLIBS)

convertCheckerpointsToRobotFrame: convertCheckerpointsToRobotFrame.cpp checkerboard_data_vis.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o
	$(LD) $(INCLUDES) $(DEBUG) -o convertCheckerpointsToRobotFrame convertCheckerpointsToRobotFrame.cpp checkerboard_data_vis.o capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o $(LIBS) 


################################################
#    Object files                              #
################################################

CannyOrient.o: $(VISION_DIR)/CannyOrient.cpp $(VISION_DIR)/CannyOrient.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(VISION_DIR)/CannyOrient.cpp
	
utils2.o: $(VISION_DIR)/utils2.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c $(VISION_DIR)/utils2.cpp

capture_gige2.o: $(VISION_DIR)/capture_gige2.cpp $(VISION_DIR)/capture2.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(VISION_DIR)/capture_gige2.cpp 
	
utility.o: $(UTILS_DIR)/utility.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c $(UTILS_DIR)/utility.cpp

clock.o: $(UTILS_DIR)/clock.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c $(UTILS_DIR)/clock.cpp
	
#blocked.o : $(VISION_DIR)/blocked.cpp blocked.h
#	$(CC) $(INCLUDES) $(DEBUG) -c $(VISION_DIR)/blocked.cpp
	
UDPSocket.o: $(SOCKET_DIR)UDPSocket.cpp 
	$(CC) $(INCLUDES) $(DEBUG) -c $(SOCKET_DIR)UDPSocket.cpp
	
messaging.o: $(MESSAGING_DIR)messaging.cc
	$(CC) $(INCLUDES) $(DEBUG) -c $(MESSAGING_DIR)messaging.cc

StereoOnClicks2.o: $(VISION_DIR)/StereoOnClicks2.cpp 
	$(CC) $(INCLUDES) $(DEBUG) -c $(VISION_DIR)/StereoOnClicks2.cpp	

ThreeCam.o: $(VISION_DIR)/ThreeCam.cpp $(VISION_DIR)/ThreeCam.h
	$(CC) $(INCLUDES) $(DEBUG) -c $(VISION_DIR)/ThreeCam.cpp

checkerboard_data_vis.o: checkerboard_data_vis.cpp  checkerboard_data_vis.h
	$(CC) $(INCLUDES) $(DEBUG) -c checkerboard_data_vis.cpp

thread_utils.o: $(THREAD_DIR)/thread_utils.cpp
	$(CC) -DHAVE_NAMESPACES $(INCLUDES) $(DEBUG) -c $(THREAD_DIR)/thread_utils.cpp

clean:
	rm *.o testCheckerboardFinder saveDataForCalib runCalibFromFiles


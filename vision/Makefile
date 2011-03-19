UTILS_DIR = ../../utils/
MESSAGING_DIR = /home/pabbeel/code/master-slave/slaves/
SOCKET_DIR = /home/pabbeel/code/master-slave/
THREAD_DIR = ../threadenergy/
LIBS = -L/usr/local/lib/ -lcxcore -lcv -lhighgui -lcvaux -lml -lm  -L/usr/lib/  -lPvAPI -lrt -L/usr/local/lib64
OPTLIBS = -L/usr/local/lib/ -lopt -lnewmat -L/usr/lib/ -lglut -lgle
#FLIBS =  -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1 -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1/../../../../lib -L/lib/../lib -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1/../../.. -L/usr/lib/x86_64-linux-gnu -lgfortranbegin -lgfortran
FLIBS =  -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1 -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1/../../../../lib -L/lib/../lib -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/4.4.1/../../.. -L/usr/lib/x86_64-linux-gnu -lgfortranbegin -lgfortran
INCLUDES = -I/usr/local/include/ -I/usr/local/include/opencv/ -I/usr/local/include/libPvAPI -I $(UTILS_DIR)
OPT_DEFINES = -DHAVE_NAMESPACES #-DWITH_MPI -DMPICH_IGNORE_CXX_SEEK
CC = g++-4.4
LD = g++-4.4
DEBUG = -ggdb

#target : testCanny testCapture example_capture_3cam calibrate_robots test_point_distances PositionsOnRequest PositionsOnRequest2 threadStereo

target: capture_3cam_continuous capture_3cam_continuous_bytesOnly imFile_to_ims read_timed_comments threadStereoReal

testCanny: testCanny.cpp CannyOrient.o
	$(LD) $(INCLUDES) $(DEBUG) -o testCanny testCanny.cpp CannyOrient.o $(LIBS)
	
testCapture: testCapture.cpp
	$(LD) $(INCLUDES) $(DEBUG) -o testCapture testCapture.cpp $(LIBS)

example_capture_3cam: capture_gige2.o example_capture_3cam.cpp utils2.o clock.o
	$(LD) $(INCLUDES) $(DEBUG) -o example_capture_3cam capture_gige2.o example_capture_3cam.cpp utils2.o clock.o $(LIBS)
	
#StereoOnClicks : capture_gige2.o StereoOnClicks.cpp utils2.o clock.o StereoOnClicks.h messaging.o UDPSocket.o
	#$(LD) $(INCLUDES) $(DEBUG) -o StereoOnClicks capture_gige2.o StereoOnClicks.cpp utils2.o clock.o messaging.o UDPSocket.o $(MESSAGING_DIR)debugging.cc $(LIBS)
	
calibrate_robots: calibrate_robots.cpp capture_gige2.o utils2.o clock.o UDPSocket.o StereoOnClicks.o messaging.o 
	$(LD) $(INCLUDES) $(DEBUG) -o calibrate_robots calibrate_robots.cpp capture_gige2.o utils2.o clock.o UDPSocket.o StereoOnClicks.o messaging.o $(MESSAGING_DIR)debugging.cc $(LIBS)
	
test_point_distances: test_point_distances.cpp capture_gige2.o utils2.o clock.o StereoOnClicks.o 
	$(LD) $(INCLUDES) $(DEBUG) -o test_point_distances test_point_distances.cpp capture_gige2.o utils2.o clock.o StereoOnClicks.o $(LIBS)	
	
PositionsOnRequest: PositionsOnRequest.cpp capture_gige2.o utils2.o clock.o UDPSocket.o StereoOnClicks.o messaging.o 
	$(LD) $(INCLUDES) $(DEBUG) -o PositionsOnRequest PositionsOnRequest.cpp capture_gige2.o utils2.o clock.o UDPSocket.o StereoOnClicks.o messaging.o $(MESSAGING_DIR)debugging.cc $(LIBS)
	
PositionsOnRequest2: PositionsOnRequest2.cpp capture_gige2.o utils2.o clock.o UDPSocket.o StereoOnClicks2.o messaging.o 
	$(LD) $(INCLUDES) $(DEBUG) -o PositionsOnRequest2 PositionsOnRequest2.cpp capture_gige2.o utils2.o clock.o UDPSocket.o StereoOnClicks2.o messaging.o $(MESSAGING_DIR)debugging.cc $(LIBS)

threadStereo: testThreadStereo.cpp ThreadStereo_Optimization.o capture_gige2.o utils2.o clock.o CannyOrient.o thread_utils.o thread_minenergy.o threadpiece_minenergy.o thread_vision.o threadpiece_vision.o ThreeCam.o StereoOnClicks2.o
	$(LD) $(OPT_DEFINES) $(INCLUDES) $(DEBUG) -o threadStereo testThreadStereo.cpp ThreadStereo_Optimization.o capture_gige2.o utils2.o clock.o CannyOrient.o thread_utils.o thread_minenergy.o threadpiece_minenergy.o thread_vision.o threadpiece_vision.o ThreeCam.o StereoOnClicks2.o  $(LIBS) $(OPTLIBS) $(FLIBS)

threadStereoReal: testThreadStereoRealIms.cpp ThreadStereo_Optimization.o capture_gige2.o utils2.o clock.o CannyOrient.o thread_utils.o thread_minenergy.o threadpiece_minenergy.o thread_vision.o threadpiece_vision.o ThreeCam.o StereoOnClicks2.o trajectory_recorder.o $(THREAD_DIR)/globals_thread_param_estimation.h
	$(LD) $(OPT_DEFINES) $(INCLUDES) $(DEBUG) -o threadStereoReal testThreadStereoRealIms.cpp ThreadStereo_Optimization.o capture_gige2.o utils2.o clock.o CannyOrient.o thread_utils.o thread_minenergy.o threadpiece_minenergy.o thread_vision.o threadpiece_vision.o ThreeCam.o StereoOnClicks2.o trajectory_recorder.o  $(LIBS) $(OPTLIBS) $(FLIBS)

capture_3cam_continuous: capture_3cam_continuous.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o 
	$(LD) $(INCLUDES) $(DEBUG) -o capture_3cam_continuous capture_3cam_continuous.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o $(LIBS) 

capture_3cam_continuous_bytesOnly: capture_3cam_continuous_bytesOnly.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o 
	$(LD) $(INCLUDES) $(DEBUG) -o capture_3cam_continuous_bytesOnly capture_3cam_continuous_bytesOnly.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o $(LIBS) 

imFile_to_ims: imFile_to_ims.cpp utils2.o clock.o
	$(LD) $(INCLUDES) $(DEBUG) -o imFile_to_ims imFile_to_ims.cpp utils2.o clock.o $(LIBS) 

capture_3cam_video: capture_3cam_video.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o 
	$(LD) $(INCLUDES) $(DEBUG) -o capture_3cam_video capture_3cam_video.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o $(LIBS) 

video_to_ims: video_to_ims.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o 
	$(LD) $(INCLUDES) $(DEBUG) -o video_to_ims video_to_ims.cpp capture_gige2.o utils2.o clock.o CannyOrient.o ThreeCam.o StereoOnClicks2.o $(LIBS) 

timed_comments: timed_comments.cpp utils2.o clock.o
	$(LD) $(INCLUDES) $(DEBUG) -o timed_comments timed_comments.cpp utils2.o clock.o $(LIBS) 

read_timed_comments: read_timed_comments.cpp utils2.o clock.o
	$(LD) $(INCLUDES) $(DEBUG) -o read_timed_comments read_timed_comments.cpp utils2.o clock.o $(LIBS) 

reprojOnIms: reprojThreadPointsOnIms.cpp ThreadStereo_Optimization.o capture_gige2.o utils2.o clock.o CannyOrient.o thread_utils.o thread_minenergy.o threadpiece_minenergy.o thread_vision.o threadpiece_vision.o ThreeCam.o StereoOnClicks2.o trajectory_recorder.o trajectory_reader.o $(THREAD_DIR)/globals_thread_param_estimation.h
	$(LD) $(OPT_DEFINES) $(INCLUDES) $(DEBUG) -o reprojOnIms reprojThreadPointsOnIms.cpp ThreadStereo_Optimization.o capture_gige2.o utils2.o clock.o CannyOrient.o thread_utils.o thread_minenergy.o threadpiece_minenergy.o thread_vision.o threadpiece_vision.o ThreeCam.o StereoOnClicks2.o trajectory_recorder.o trajectory_reader.o  $(LIBS) $(OPTLIBS) $(FLIBS)

calculateReprojError: calculate_reproj_error.cpp ThreadStereo_Optimization.o capture_gige2.o utils2.o clock.o CannyOrient.o thread_utils.o thread_minenergy.o threadpiece_minenergy.o thread_vision.o threadpiece_vision.o ThreeCam.o StereoOnClicks2.o trajectory_recorder.o trajectory_reader.o $(THREAD_DIR)/globals_thread_param_estimation.h
	$(LD) $(OPT_DEFINES) $(INCLUDES) $(DEBUG) -o calculateReprojError calculate_reproj_error.cpp ThreadStereo_Optimization.o capture_gige2.o utils2.o clock.o CannyOrient.o thread_utils.o thread_minenergy.o threadpiece_minenergy.o thread_vision.o threadpiece_vision.o ThreeCam.o StereoOnClicks2.o trajectory_recorder.o trajectory_reader.o  $(LIBS) $(OPTLIBS) $(FLIBS)

################################################
#    Object files                              #
################################################
CannyOrient.o : CannyOrient.cpp CannyOrient.h
	$(CC) $(INCLUDES) $(DEBUG) -c CannyOrient.cpp
	
utils2.o : utils2.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c utils2.cpp

thread_utils.o: $(THREAD_DIR)/thread_utils.cpp
	$(CC) $(OPT_DEFINES) $(INCLUDES) $(DEBUG) -c $(THREAD_DIR)/thread_utils.cpp $(OPTLIBS) $(FLIBS)
	
capture_gige2.o : capture_gige2.cpp capture2.h
	$(CC) $(INCLUDES) $(DEBUG) -c capture_gige2.cpp
	
utility.o : $(UTILS_DIR)/utility.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c $(UTILS_DIR)/utility.cpp

clock.o : $(UTILS_DIR)/clock.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c $(UTILS_DIR)/clock.cpp
	
blocked.o : blocked.cpp blocked.h
	$(CC) $(INCLUDES) $(DEBUG) -c blocked.cpp
	
UDPSocket.o: $(SOCKET_DIR)UDPSocket.cpp 
	$(CC) $(INCLUDES) $(DEBUG) -c $(SOCKET_DIR)UDPSocket.cpp
	
messaging.o: $(MESSAGING_DIR)messaging.cc
	$(CC) $(INCLUDES) $(DEBUG) -c $(MESSAGING_DIR)messaging.cc

StereoOnClicks.o: StereoOnClicks.cpp 
	$(CC) $(INCLUDES) $(DEBUG) -c StereoOnClicks.cpp
	
StereoOnClicks2.o: StereoOnClicks2.cpp 
	$(CC) $(INCLUDES) $(DEBUG) -c StereoOnClicks2.cpp	

ThreeCam.o: ThreeCam.cpp ThreeCam.h
	$(CC) $(INCLUDES) $(DEBUG) -c ThreeCam.cpp

ThreadStereo_SpaceCurve.o: ThreadStereo_SpaceCurve.cpp
	$(CC) $(INCLUDES) $(DEBUG) -c ThreadStereo_SpaceCurve.cpp

ThreadStereo_SpaceCurve_MinEnergy.o: ThreadStereo_SpaceCurve_MinEnergy.cpp
	$(CC) $(OPT_DEFINES) $(INCLUDES) $(DEBUG) -c ThreadStereo_SpaceCurve_MinEnergy.cpp $(OPTLIBS) $(FLIBS)

ThreadStereo_Optimization.o: ThreadStereo_Optimization.cpp ThreadStereo_Optimization.h thread_vision.h
	$(CC) $(OPT_DEFINES) $(INCLUDES) $(DEBUG) -c ThreadStereo_Optimization.cpp $(OPTLIBS) $(FLIBS)

ThreadStereo_SpaceCurve_OptimizePoints.o: ThreadStereo_SpaceCurve_OptimizePoints.cpp
	$(CC) $(OPT_DEFINES) $(INCLUDES) $(DEBUG) -c ThreadStereo_SpaceCurve_OptimizePoints.cpp $(OPTLIBS) $(FLIBS)

thread_minenergy.o : $(THREAD_DIR)/thread_minenergy.cpp
	$(CC) $(OPT_DEFINES) $(INCLUDES) $(DEBUG) -c $(THREAD_DIR)/thread_minenergy.cpp $(OPTLIBS) $(FLIBS)

threadpiece_minenergy.o: $(THREAD_DIR)/threadpiece_minenergy.cpp
	$(CC) $(OPT_DEFINES) $(INCLUDES) $(DEBUG) -c $(THREAD_DIR)/threadpiece_minenergy.cpp $(OPTLIBS) $(FLIBS)

thread_vision.o: thread_vision.cpp thread_vision.h
	$(CC) $(OPT_DEFINES)  $(INCLUDES) $(DEBUG) -c thread_vision.cpp $(OPTLIBS) $(FLIBS)

threadpiece_vision.o: threadpiece_vision.cpp 
	$(CC) $(OPT_DEFINES) $(INCLUDES) $(DEBUG) -c threadpiece_vision.cpp $(OPTLIBS) $(FLIBS)

trajectory_recorder.o : $(THREAD_DIR)/trajectory_recorder.cpp
	$(CC) $(OPT_DEFINES) $(INCLUDES) $(DEBUG) -c $(THREAD_DIR)/trajectory_recorder.cpp

trajectory_reader.o : $(THREAD_DIR)/trajectory_reader.cpp
	$(CC) $(OPT_DEFINES) $(INCLUDES) $(DEBUG) -c $(THREAD_DIR)/trajectory_reader.cpp


clean:
	rm *.o testCanny testCapture example_capture_3cam calibrate_robots test_point_distances PositionsOnRequest PositionsOnRequest2 threadStereo threadStereoReal



CXX = g++
CFLAGS = -I../ -I../../cwmtx-0.6.2/ -I../../ -I../../../../utils/ -I/usr/include/ -I/usr/local/include -I/usr/include/eigen2/
LDFLAGS = -lGalil
FULL_OBJS = ../util.cc ../../Socket.cpp ../../UDPSocket.cpp ../controller.cc ../controller_pid.cc ../controller_dummy.cc ../controller_generic.cc ../controller_lqr.cc ../controller_lqr_and_pid.cc ../master.cc ../../../../utils/utility.cpp ../../../../utils/LowPassFilter.cpp ../slave.cc ../slave_mark1.cc ../slave_accelera.cc ../system.cc ../system_accelera.cc ../system_mark1.cc ../system_generic.cc ../actuator.cc ../sensor.cc ../miranova.cc ../quatech.cc ../debugging.cc ../motion_planner.cc ../interpolate_planner.cc ../controller_visual.cc ../controller_visual_servo.cc ../controller_motion_planner.cc ../accelera_geometry.cc ../visual_feedback.cc ../messaging.cc ../controller_multi_planner.cc ../motion_planning_segment.cc ../visual_segment.cc ../non_visual_segment.cc ../controller_visual_knottie.cc
# ../accelera_record.cc ../accelera_encoder.cc ../accelera_actuator.cc 
SERIALLIB = -L/usr/local/lib/ -lezV24 -lboost_system 
KDLLIB = -L/usr/lib/ -lorocos-kdl
default: setpoints

encoders: test_encoders.cpp
	$(CXX) $(CFLAGS) test_encoders.cpp -o encoders $(LDFLAGS) $(FULL_OBJS)

actuators: test_actuators.cpp
	$(CXX) $(CFLAGS) test_actuators.cpp -o actuators $(LDFLAGS) $(FULL_OBJS)

setpoints: test_setpoints.cpp
	$(CXX) $(CFLAGS) test_setpoints.cpp -o setpoints $(LDFLAGS) $(FULL_OBJS) $(KDLLIB)

servo_joysticks: test_setpoints_servos.cpp
	$(CXX) $(CFLAGS) test_setpoints_servos.cpp -o servo_joysticks $(LDFLAGS) $(FULL_OBJS) ../Servo_Controller.cpp $(SERIALLIB)

digital_out: test_digital_out.cpp
	$(CXX) $(CFLAGS) test_digital_out.cpp -o digital_out $(LDFLAGS) $(FULL_OBJS)

location: test_location.cpp
	$(CXX) $(CFLAGS) test_location.cpp -o location $(LDFLAGS) $(FULL_OBJS)

tune_pid: tune_pid.cpp
	$(CXX) $(CFLAGS) tune_pid.cpp -o tune_pid $(LDFLAGS) $(FULL_OBJS)

replay: replay.cpp
	$(CXX) $(CFLAGS) replay.cpp -o replay $(LDFLAGS) $(FULL_OBJS)

accelera: test_accelera.cpp
	$(CXX) $(CFLAGS) test_accelera.cpp -o accelera $(LDFLAGS) $(FULL_OBJS)

map_xyz: map_xyz.cc
	$(CXX) $(CFLAGS) map_xyz.cc -o map_xyz $(LDFLAGS) $(FULL_OBJS)

kinem: test_kinem.cc
	$(CXX) $(CFLAGS) test_kinem.cc -o kinem $(LDFLAGS) $(FULL_OBJS)

calibrate: test_calibrate_cameras.cpp
	$(CXX) $(CFLAGS) test_calibrate_cameras.cpp -o calibrate $(LDFLAGS) $(FULL_OBJS)

calibration: test_calibration.cpp
	$(CXX) $(CFLAGS) test_calibration.cpp -o calibration $(LDFLAGS) $(FULL_OBJS) $(KDLLIB)

network: test_network_masters.cpp
	$(CXX) $(CFLAGS) test_network_masters.cpp -o network $(LDFLAGS) $(FULL_OBJS)
    
    
    
#################################################################
#                         Object Files                          #    
#################################################################
util.o : ../util.cc
	$(CXX) $(CFLAGS) -c ../util.cc



 
clean:
	rm -rf ../*.o *.o encoders actuators setpoints servo_joysticks heartbeat location tune_pid replay digital_out accelera map_xyz kinem calibrate network calibration

# must run image_proc_3cam.sh prior so rectified image topics in
#   rostopics list
roslaunch camera_pose_calibration calibrate_3_camera.launch camera1_ns:=prosilica/left camera2_ns:=prosilica/right camera3_ns:=prosilica/top checker_rows:=7 checker_cols:=6 checker_size:=0.0125

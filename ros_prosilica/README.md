> This folder contains code for using [ROS](http://www.ros.org) packages for
> interfacing with the Prosilica GC1290 cameras. ROS provides a very handy
> package within the `camera_drivers` stack for [Prosilica cameras](http://www.ros.org/wiki/prosilica_camera).

## Calibration

Camera calibration uses two packages, `camera_calibration` (for intrinsics) and
`camera_pose_calibration`. The current scripts use a checkerboard of size 8x7
with 0.0125m squares.

To perform calibration, in the command line, follow these steps:

1. Start `roscore`.
2. Within  `surgical/ros_prosilica`, run `roslaunch prosilica.launch`.
3. In the same directory run `./image_proc_3cam.sh`. This sets up the rectified
   images necessary for pose calibration.
4. Follow the tutorials on [camera calibration](http://www.ros.org/wiki/camera_calibration)
   and [camera pose calibration](http://www.ros.org/wiki/camera_pose_calibration)
   on the ROS wiki. You may want to examine `rostopic list` and use the shell
   scripts provided.
5. The camera calibration output is easy to copy. For pose calibration,
   run `rostopic echo /camera_calibration` to obtain the output w/o dealing w/
   [tf](http://www.ros.org/wiki/tf). *#TODO: figure out better extraction method
   for automation*.

## Streaming

After running `roslaunch prosilica.launch`, start
`rosrun mjpeg_server mjpeg_server`. Then visit the stream by following the
instructions at the ROS [mjpeg_server page](http://www.ros.org/wiki/mjpeg_server).
If there is an error accessing the stream, try setting the pixel type in
SampleViewer to `Bgr24`. You can also [dynamically reconfigure](http://www.ros.org/wiki/dynamic_reconfigure)
some options. *#TODO: Fix frame rate and video quality, improve video compression*.

## Debugging

Currently Diamondback has a few issues, oftentimes the solution is to copy
relevant ROS packages to home directory, edit the manifest or C++/Python files
to enable/fix desired functionality, add folder to start of `ROS_PACKAGE_PATH`
, and run `rosmake`.


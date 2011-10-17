# usage: ./camera_calibration.sh [left/right/top]
rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.0125 image:=/prosilica/$1/image_raw camera:=/prosilica/$1

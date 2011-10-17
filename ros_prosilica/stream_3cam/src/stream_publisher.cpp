/*
 * Simple image publisher for the Prosilica 3-camera set-up.
 * Refer to http://www.ros.org/wiki/image_transport.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

int main(int argc, char** argv)
{
  // Pass command line arguments and specify node name.
  ros::init(argc, argv, "stream_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  // Publishing on topic "camera/image".
  // Second argument specifies size of publishing queue.
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  cv::WImageBuffer3_b image(cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR));
  sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(),
    "bgr8");

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

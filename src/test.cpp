#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub;
  ros::Publisher pos_pub;
  ros::Subscriber depth_pos_sub;

private :
  // Erode the image
  cv::Mat eroded;

  // Dilate the image
  cv::Mat dilated;
  cv::Mat clone_eroded;
  cv::Mat origin_result;

public:
  cv::Mat threshold_frame;
  double dist;
  int width_center;
  int robot_posX;
  int robot_posY;
  int minDetect;
  bool detection;


  ImageConverter()

  {
    ros::Time::init();
    ros::Duration du(5.0);
    du.sleep();
    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
    pos_pub = nh_.advertise<geometry_msgs::Pose>("ball_info_pose",1);
    //depth_pos_sub = nh_.subscribe("depth_info_dist", 1, &ImageConverter::depthInfoPoseCb, this);

  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }


  void test()
  {
    int a = NULL;
    std::cout << a << std::endl;

  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  ImageConverter ic;

  while(ros::ok()) {
    ic.test();
  }


  ros::spin();
  return 0;
}


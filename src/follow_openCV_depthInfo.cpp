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

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub;
  ros::Publisher pos_pub;
  ros::Subscriber depth_pos_sub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

private :
  // Erode the image
  cv::Mat eroded;
  // Dilate the image
  cv::Mat dilated;
  cv::Mat clone_eroded;
  cv::Mat origin_result;

public:
  cv::Mat threshold_frame;
  double opencv_Distance;
  double depth_Distance;     // transmitted from depth_info node
  double minDetect;
  bool detection;
  int width_center;
  int robot_posX;
  int robot_posY;

  ImageConverter()
    : it_(nh_), depth_Distance(0), minDetect(0.80), width_center(320), robot_posX(0), robot_posY(0)
  {
    ros::Duration du(2.0);
    du.sleep();

    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("camera/rgb/image_color", 1, &ImageConverter::imageCb, this);

    pos_pub = nh_.advertise<geometry_msgs::Pose>("ball_info_pose",1);                                   // publish ball position
    depth_pos_sub = nh_.subscribe("depth_info_dist", 1, &ImageConverter::depthInfoDistCb, this);        // subscribe distance
    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);                                     // publish velocity for following motion
  }

  cv::Mat threshold(const cv::Mat& frame)
  {
    cv::Mat result1;    // minimum range
    cv::Mat result2;    // maximum range
    cv::Mat result;
    cv::Mat converted;
    converted.create(frame.rows, frame.cols, frame.type());
    cv::cvtColor(frame, converted, CV_BGR2HSV);
    result1.create(frame.rows, frame.cols, CV_8U);
    result2.create(frame.rows, frame.cols, CV_8U);
    result.create(frame.rows, frame.cols, CV_8U);
    cv::inRange(converted, cv::Scalar(0,200,120), cv::Scalar(2,255,255), result1);
    cv::inRange(converted, cv::Scalar(170,200,120),cv::Scalar(179,255,255),result2);
    cv::bitwise_or(result1, result2, result);

    return result;
  }

  void cmd_vel_command()
  {
    geometry_msgs::Twist cmd;

    if (depth_Distance < 2 && depth_Distance > minDetect) {
      cmd.linear.x = depth_Distance-0.95;
      if (robot_posX > width_center) {
        cmd.angular.z = -fabs(robot_posX - width_center)/1000;
      } else if (robot_posX < width_center) {
        cmd.angular.z = fabs(robot_posX - width_center)/1000;
      } else if (robot_posX == width_center) {
        cmd.angular.z = 0.0;
      }
    }
    else if (depth_Distance < minDetect && depth_Distance > 0.65) {
      std::cout << "depth_Distance : " << depth_Distance << " " << "move to back..." << std::endl;
      cmd.linear.x = depth_Distance-1.1;	
    }
    else if (depth_Distance < 0.65) {
      std::cout << "depth_Distance : " << depth_Distance << " " << "stop command" << std::endl;
      cmd_vel_pub.publish(geometry_msgs::Twist()); // zero msg
    }
    cmd_vel_pub.publish(cmd);
    printf("linear vel : %.4f angular_vel : %.4f\n", cmd.linear.x, cmd.angular.z);
    printf("\n");
  }

  // This call back function received the data which is distance from depth_info(node) using kinect
  // Distance is more exact than distance from opencv
  // We'll use depth_Distance variable to following motion
  void depthInfoDistCb(const geometry_msgs::Pose& distance)
  {
    depth_Distance = distance.position.z;

    if(!std::isnan(depth_Distance)) {
      std::cout << "find a ball & publish ball pose" << std::endl;
      std::cout << "x : " << robot_posX << " y : " << robot_posY << " depth info distance : " << depth_Distance << std::endl;
      cmd_vel_command();
    } else {
      std::cout << "distance value : [nan]" << std::endl;
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // function call
    threshold_frame = threshold(cv_ptr->image);

    // MORPH_ELLIPSE=2
    cv::Mat kernel = cv::getStructuringElement(2, cv::Size( 3, 3 ), cv::Point( -1, -1 ));

    // Dilate the frame
    cv::dilate(threshold_frame, dilated, kernel, cv::Point(-1,-1), 5);

    // Erode the frame
    cv::erode(dilated, eroded, kernel, cv::Point(-1,-1), 3);

    // for saving the eroded frame
    clone_eroded = eroded.clone();

    std::vector< std::vector<cv::Point> > contours;     // storage for the contours
    std::vector<cv::Vec4i> hierarchy;                   // hierachyminDetect

    // just get the contours
    // using clone_eroded image
    cv::findContours( clone_eroded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0) );

    int no_ellipse(0);
    for( int i(0); i< (contours.size()); i++ )
    {
      if( contours[i].size() < 25 ) continue;
      cv::drawContours( cv_ptr->image, contours, i, cv::Scalar(255,0,0), 1, 8 );
      cv::Moments mom = cv::moments( cv::Mat(contours[i]));
      double area = mom.m00;
      double perimeter = cv::arcLength(cv::Mat(contours[i]),true);
      double circularity = 4*CV_PI*area/(perimeter*perimeter);

      // find a ball
      if( circularity > 0.5 )
      {
        cv::RotatedRect ellipse_candidate = cv::fitEllipse( cv::Mat(contours[i]) );
        cv::ellipse( cv_ptr->image, ellipse_candidate, cv::Scalar(0,255,0), 2, 8 );
        no_ellipse ++;

        // ball centroid
        cv::circle(cv_ptr->image, cv::Point(mom.m10/mom.m00, mom.m01/mom.m00), 10, cv::Scalar(0.8, 0.2, 0.2), 2);

        static int posX = 0;
        static int posY = 0;

        posX = mom.m10/mom.m00;
        posY = mom.m01/mom.m00;

        robot_posX = posX;
        robot_posY = posY;
      }
      //std::cout << "circularity " << circularity << std::endl;
      //std::cout << "we have " << contours.size() << " contours --> " << no_ellipse << " found" << std::endl;
      //std::cout << " x : " << robot_posX << " y : " << robot_posY << " opencv_Distance : " << opencv_Distance << std::endl;
    }

    // ball position publish
    if (no_ellipse > 0 && no_ellipse < 2) {
      geometry_msgs::Pose pos;
      pos.position.x = robot_posX;
      pos.position.y = robot_posY;
      pos_pub.publish(pos);
      //std::cout << "find a ball & publish ball pose : " << pos.position.x << " , " << pos.position.y << std::endl;
    }

    // can't find a ball
    else {
      robot_posX = 0;
      robot_posY = 0;
      std::cout << "kobuki can't find the ball" << std::endl;
    }

    cv::imshow("origin", cv_ptr->image);
    cv::imshow("threshold", threshold_frame);
    //cv::imshow("dilate", dilated);
    //cv::imshow("erode", eroded);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());


  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;


  ros::spin();
  return 0;
}

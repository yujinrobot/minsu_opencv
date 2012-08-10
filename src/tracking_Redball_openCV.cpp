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
  bool detection;
  int width_center;
  int robot_posX;
  int robot_posY;
  int minDetect;
  int flag;


  ImageConverter()
    : it_(nh_), depth_Distance(0), width_center(320), robot_posX(0), robot_posY(0), minDetect(0.85), flag(0)
  {
    ros::Time::init();
    ros::Duration du(5.0);
    du.sleep();

    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
    //image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this, image_transport::TransportHints("compressed"));

    pos_pub = nh_.advertise<geometry_msgs::Pose>("ball_info_pose",1);                                   // publish ball position
    depth_pos_sub = nh_.subscribe("depth_info_dist", 1, &ImageConverter::depthInfoPoseCb, this);        // subscribe distance
    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);                                     // publish velocity for following motion
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
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
    cv::inRange(converted, cv::Scalar(0,160,150), cv::Scalar(4,255,255), result1);
    cv::inRange(converted, cv::Scalar(170,160,180),cv::Scalar(179,255,255),result2);
    cv::bitwise_or(result1, result2, result);

    //cv::imshow("converted", converted);

    /*
    // for RGB color space
    cv::Mat_<cv::Vec3b>::const_iterator it_rgb= frame.begin<cv::Vec3b>();

    // for HSV color space
    cv::Mat_<cv::Vec3b>::const_iterator it= converted.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::const_iterator itend= converted.end<cv::Vec3b>();
    cv::Mat_<uchar>::iterator itout = result.begin<uchar>();

    // RGB color space
    int avg_red = 0;
    int avg_green = 0;
    int avg_blue = 0;


    // HSV color space
    int avg_hue = 0, oor = 0;
    int avg_saturation = 0;
    int avg_value = 0;

    for(; it!=itend; ++it, ++itout) {

      // RGB color space
      avg_blue += (*it_rgb)[0];
      avg_green += (*it_rgb)[1];
      avg_red += (*it_rgb)[2];

      // HSV color space
      // range Hue              : 0 ~ 360
      // range Saturation       : 0 ~ 100
      // range Value(intensity) : 0 ~ 100

      // 1 byte                 : 0 ~ 25
      // real range Hue         : 0 ~ 180 (Hue/2)

      avg_hue += (*it)[0];              // hue value
      avg_saturation += (*it)[1];       // saturation value
      avg_value += (*it)[2];            // value value

      if ((*it)[0] > 180) oor++;

    }

    int avg_hue2 = 0;
    int avg_saturation2 = 0;
    int avg_value2 = 0;

    cv::Rect roi = cv::Rect(320-40, 240-30, 80, 60);
    cv::Mat cropped = converted(roi);
i
    it= cropped.begin<cv::Vec3b>();
    itend= cropped.end<cv::Vec3b>();

    for(; it!=itend; ++it, ++itout) {
      avg_hue2 += (*it)[0];              // hue value
      avg_saturation2 += (*it)[1];       // saturation value
      avg_value2 += (*it)[2];            // value value

    }

    std::cout << "R : " << avg_red/frame.size().area() << " " << "G : " << avg_green/frame.size().area() << " "
              << "B : " << avg_blue/frame.size().area()<< std::endl;
    //std::cout << avg_hue/converted.size().area() << "  " <<  oor << "\n";
    std::cout << "H : " << avg_hue/converted.size().area() << " " << "S : " << avg_saturation/converted.size().area() << " "
              << "V : " << avg_value/converted.size().area() << std::endl << std::endl;

    std::cout << "H : " << avg_hue2/cropped.size().area() << " " << "S : " << avg_saturation2/cropped.size().area() << " "
              << "V : " << avg_value2/cropped.size().area() << std::endl << std::endl;
    */

    return result;
  }

  // This call back function received the data which is distance from depth_info(node) using kinect
  // Distance is more exact than distance from opencv
  // We'll use depth_Distance variable to following motion
  void depthInfoPoseCb(const geometry_msgs::Pose& distance)
  {
    depth_Distance = distance.position.z;

    if(!std::isnan(depth_Distance)) {
      std::cout << "depth info distance : " << depth_Distance << std::endl;
    } else {
      std::cout << "distance value : [nan]" << std::endl;
    }
  }

  void cmd_vel_command()
  {
    geometry_msgs::Twist cmd;

    if (flag == 1)
    {
      if (depth_Distance < 2 && depth_Distance > minDetect) {
        //cmd.linear.x = depth_Distance-0.5;
        cmd.linear.x = 0.3;
        if (robot_posX > width_center) {
          //cmd.angular.z = -fabs(robot_posX - width_center)/100;
          cmd.angular.z = -0.3;
        } else if (robot_posX < width_center) {
          //cmd.angular.z = fabs(robot_posX - width_center)/100;
          cmd.angular.z = 0.3;
        } else if (robot_posX == width_center) {
          cmd.angular.z = 0.0;
        }
        cmd_vel_pub.publish(cmd);
        std::cout << "depth_Distance : " << depth_Distance << " " << "detecting ball" << " " << "vel_x : " << cmd.linear.x << " " << "vel_z" << " " << cmd.angular.z << std::endl;
      }

      else if (depth_Distance < minDetect && depth_Distance > 0.65) {
        std::cout << "depth_Distance : " << depth_Distance << " " << "stop command" << std::endl;
        cmd_vel_pub.publish(geometry_msgs::Twist()); // zero msg
        //depth_Distance = 0;
      }
    }

    // flag = 0
    else
    {
      cmd.angular.z = 0.5;
      cmd_vel_pub.publish(cmd);
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //ros::Time start = ros::Time::now();
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


    /*
    // pre-smoothing improves Hough detector
    cv::GaussianBlur(clone_eroded, clone_eroded, cv::Size(3,3), 1.5);

    // hough circle algorithm
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(clone_eroded, circles, CV_HOUGH_GRADIENT, 2, 200, 200, 100, 25, 100);
    std::vector<cv::Vec3f>::const_iterator itc = circles.begin();

    while(itc!=circles.end()) {
      cv::circle(cv_ptr->image, cv::Point( (*itc)[0], (*itc)[1]) , (*itc)[2], cv::Scalar(255), 2);
      ++itc;
    }
    */


    std::vector< std::vector<cv::Point> > contours;     // storage for the contours
    std::vector<cv::Vec4i> hierarchy;                   // hierachyminDetect

    // for saving the result frame
    //origin_result = result.clone();

    // just get the contours
    // using clone_eroded image
    cv::findContours( clone_eroded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0) );

    int no_ellipse(0);
    for( int i(0); i< (contours.size()); i++ )
    {
      if( contours[i].size() < 25 ) continue;
      cv::drawContours( cv_ptr->image, contours, i, cv::Scalar(255,0,0), 1, 8 );
      cv::Moments moms = cv::moments( cv::Mat(contours[i]));
      double area = moms.m00;
      double perimeter = cv::arcLength(cv::Mat(contours[i]),true);
      double circularity = 4*CV_PI*area/(perimeter*perimeter);

      // find a ball
      if( circularity > 0.5 )
      {
        cv::RotatedRect ellipse_candidate = cv::fitEllipse( cv::Mat(contours[i]) );
        cv::ellipse( cv_ptr->image, ellipse_candidate, cv::Scalar(0,255,0), 2, 8 );
        no_ellipse ++;

        // radius -> {(W+H)/2}/2
        //double radius_i = (ellipse_candidate.size.height+ellipse_candidate.size.width)/4;

        //std::cout << "height : " << std::setw(7) << ellipse_candidate.size.height
        //    << " " << "width : " << std::setw(7) << ellipse_candidate.size.width
        //    << " " << "radius : " << radius_i << std::endl;

        //double f= 700;
        //opencv_Distance = 3 * f / radius_i;
        //std::cout << "kobuki find the ball | opencv_Distance : " << opencv_Distance << std::endl;
        //printf("R | Z = %f | %f\n", radius_i, double(opencv_Distance) );



        // ball centroid
        cv::Moments mom = cv::moments(threshold_frame);
        cv::circle(cv_ptr->image, cv::Point(mom.m10/mom.m00, mom.m01/mom.m00), 10, cv::Scalar(0.8, 0.2, 0.2), 2);

        static int posX = 0;
        static int posY = 0;

        posX = mom.m10/mom.m00;
        posY = mom.m01/mom.m00;

        robot_posX = posX;
        robot_posY = posY;

        // ball position publish
        geometry_msgs::Pose pos;
        pos.position.x = robot_posX;
        pos.position.y = robot_posY;
        pos_pub.publish(pos);
        std::cout << "kobuki find a ball & publish ball position to depth_info node" << std::endl;
        flag = 1;
        //std::cout << "robot_posX - width_center : " << fabs(robot_posX - width_center)/100 << std::endl;
      }

      // can't find a ball
      else {
        //opencv_Distance = 0;
        robot_posX = 0;
        robot_posY = 0;
        flag = 0;
        std::cout << "find contour but kobuki can't find the ball" << std::endl;
      }
      //std::cout << "circularity " << circularity << std::endl;
      //std::cout << "we have " << contours.size() << " contours --> " << no_ellipse << " found" << std::endl;
    }
    //std::cout << " x : " << robot_posX << " y : " << robot_posY << " opencv_Distance : " << opencv_Distance << std::endl;

    //cmd_vel_command();

    //cv::Rect rect(320-40, 240-30, 80, 60);
    //cv::rectangle(cv_ptr->image, rect, cv::Scalar(0,0,255), 5);
    cv::imshow("origin", cv_ptr->image);
    cv::imshow("threshold", threshold_frame);
    cv::imshow("dilate", dilated);
    cv::imshow("erode", eroded);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }

  /*
  // based on depth information
  void cmd_command()
  {
    geometry_msgs::Twist cmd;

    if (depth_Distance < 2 && depth_Distance > minDetect) {
      //cmd.linear.x = depth_Distance-0.5;
      cmd.linear.x = 0.3;
      if (robot_posX > width_center) {
        //cmd.angular.z = -fabs(robot_posX - width_center)/100;
        cmd.angular.z = -0.3;
      } else if (robot_posX < width_center) {
        //cmd.angular.z = fabs(robot_posX - width_center)/100;
        cmd.angular.z = 0.3;
      } else if (robot_posX == width_center) {
        cmd.angular.z = 0.0;
      }
      cmd_vel_pub.publish(cmd);
      std::cout << "depth_Distance : " << depth_Distance << " " << "detecting ball" << " " << "vel_x : " << cmd.linear.x << " " << "vel_z" << " " << cmd.angular.z << std::endl;
    }

    else if (depth_Distance < minDetect && depth_Distance > 0.65) {
      std::cout << "depth_Distance : " << depth_Distance << " " << "stop command" << std::endl;
      cmd_vel_pub.publish(geometry_msgs::Twist()); // zero msg
      depth_Distance = 0;
    }

    else {
      std::cout << "depth_Distance : " << depth_Distance << " " << "rotation" << std::endl;
      cmd.angular.z = 0.5;
      cmd_vel_pub.publish(cmd);
    }
  }
  */

  /*
  // based on openCV
  void cmd_command()
  {
    geometry_msgs::Twist cmd;

    if (opencv_Distance < 200 && opencv_Distance > minDetect) {
      cmd.linear.x = opencv_Distance/100 -0.7;
      if (robot_posX > width_center) {
        cmd.angular.z = -0.3;
      } else if (robot_posX < width_center) {
        cmd.angular.z = 0.3;
      } else if (robot_posX == width_center) {
        cmd.angular.z = 0.0;
      }
      cmd_vel_pub.publish(cmd);
      std::cout << "opencv_Distance : " << opencv_Distance << " " << "detecting ball" << " " << "vel_x : " << cmd.linear.x << " " << "vel_z" << " " << cmd.angular.z << std::endl;
    }

    else if (opencv_Distance < minDetect && opencv_Distance > 70) {
      std::cout << "opencv_Distance : " << opencv_Distance << " " << "stop command" << std::endl;
      cmd_vel_pub.publish(geometry_msgs::Twist()); // zero msg
      opencv_Distance = 0;
    }

    else {
      std::cout << "opencv_Distance : " << opencv_Distance << " " << "rotation" << std::endl;
      cmd.angular.z = 0.5;
      cmd_vel_pub.publish(cmd);
    }
  }
  */


  // It is test function.
  void test()
  {
    geometry_msgs::Twist cmd;
    cmd.angular.z = 0.2;
    cmd_vel_pub.publish(cmd);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;


  ros::spin();
  return 0;
}


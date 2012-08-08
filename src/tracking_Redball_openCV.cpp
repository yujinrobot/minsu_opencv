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

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

private :
  cv::Vec3b target;

  // Erode the image
  cv::Mat eroded;

  // Dilate the image
  cv::Mat dilated;
  cv::Mat clone_eroded;
  cv::Mat origin_result;

public:
  cv::Mat threshold_frame;
  int dist;

  ImageConverter()
    : it_(nh_), dist(0)
  {
    ros::Time::init();
    ros::Duration du(5.0);
    du.sleep();
    image_pub_ = it_.advertise("out", 1);
    //image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this, image_transport::TransportHints("compressed"));
    image_sub_ = it_.subscribe("camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
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
    cv::inRange(converted, cv::Scalar(0,100,150), cv::Scalar(2,255,255), result1);
    cv::inRange(converted, cv::Scalar(170,150,180),cv::Scalar(179,255,255),result2);
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

      // 1 byte                 : 0 ~ 255
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

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    ros::Time start = ros::Time::now();
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

    // Dilate the image
    cv::dilate(threshold_frame, dilated, kernel, cv::Point(-1,-1), 5);

    // Erode the image
    cv::erode(dilated, eroded, kernel, cv::Point(-1,-1), 2);

    clone_eroded = eroded.clone();

    std::vector< std::vector<cv::Point> > contours;     // storage for the contours
    std::vector<cv::Vec4i> hierarchy;                   // hierachy

    // for saving the result frame
    //origin_result = result.clone();

    // for saving the eroded frame200
    clone_eroded = eroded.clone();

    // just get the contours
    // using clone_eroded image
    cv::findContours( clone_eroded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0) );


    int no_ellipse(0);
    for( int i(0); i< (contours.size()); i++ )
    {
      if( contours[i].size() < 50 ) continue;
      cv::drawContours( cv_ptr->image, contours, i, cv::Scalar(255,0,0), 1, 8 );
      cv::Moments moms = cv::moments( cv::Mat(contours[i]));
      double area = moms.m00;
      double perimeter = cv::arcLength(cv::Mat(contours[i]),true);
      double circularity = 4*CV_PI*area/(perimeter*perimeter);


      if( circularity > 0.65 )
      {
        cv::RotatedRect ellipse_candidate = cv::fitEllipse( cv::Mat(contours[i]) );
        cv::ellipse( cv_ptr->image, ellipse_candidate, cv::Scalar(0,255,0), 2, 8 );
        no_ellipse ++;

        // radius -> {(W+H)/2}/2
        double radius_i = (ellipse_candidate.size.height+ellipse_candidate.size.width)/4;

        //std::cout << "height : " << std::setw(7) << ellipse_candidate.size.height
        //    << " " << "width : " << std::setw(7) << ellipse_candidate.size.width
        //    << " " << "radius : " << radius_i << std::endl;

        double f= 700;
        dist = 3 * f / radius_i;
        printf("R | Z = %f | %f\n", radius_i, double(dist) );

      }
      //std::cout << "circularity " << circularity << std::endl;
    }

    cmd_command();


    //cv::Rect rect(320-40, 240-30, 80, 60);
    //cv::rectangle(cv_ptr->image, rect, cv::Scalar(0,0,255), 5);
    cv::imshow("origin", cv_ptr->image);
    cv::imshow("threshold", threshold_frame);
    cv::waitKey(3);

    ros::Time now = ros::Time::now();
    //std::cout << "processing time : " << now - start << " sec" << std::endl;
    image_pub_.publish(cv_ptr->toImageMsg());

  }

  void cmd_command()
  {
    geometry_msgs::Twist twist;

    if (dist < 100 && dist > 0) {
      std::cout << "dist : " << dist << " " << "detecting ball & stop command" << std::endl;
      twist.angular.z = 0.0;
      cmd_vel_pub.publish(twist);
      //dist = 0;
    } else {
      twist.angular.z = 0.25;
      cmd_vel_pub.publish(twist);
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;

  ros::spin();
  return 0;
}

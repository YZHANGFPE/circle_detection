/**
 * @file HoughCircle_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

/*#include "opencv2/imgcodecs.hpp"*/
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace cv;
using namespace Eigen;

namespace
{
    // windows and trackbars name
    const std::string windowName = "Hough Circle Detection Demo";
    const std::string cannyThresholdTrackbarName = "Canny Threshold";
    const std::string accumulatorThresholdTrackbarName = "Accumulator Threshold";
    const std::string radiusThresholdTrackbarName = "Radius Threshold";
    const std::string usage = "Usage : tutorial_HoughCircle_Demo <path_to_input_image>\n";

    // initial and max values of the parameters of interests.
    const int cannyThresholdInitialValue = 133; // for table 90
    const int accumulatorThresholdInitialValue = 41; // for table 41
    const int radiusThresholdInitialValue = 45; // for table 38
    const int heightInitialValue = 26; // for table 33
    const int maxAccumulatorThreshold = 200;
    const int maxCannyThreshold = 255;
    const int maxRadiusThreshold = 100;
    const int minRadiusThreshold = 20;


    // declare and initialize both parameters that are subjects to change
    int cannyThreshold = cannyThresholdInitialValue;
    int accumulatorThreshold = accumulatorThresholdInitialValue;
    int radiusThreshold = radiusThresholdInitialValue;
    int height = heightInitialValue;

    // calibratioin matrix
    MatrixXd m(4,3);
    Vector3d v(3);
    double w = 0.2;

    ros::Publisher vis_pub; 

    void publishMarker(double x, double y, double z){
      visualization_msgs::Marker marker;
      marker.header.frame_id = "left_hand_camera";
      marker.header.stamp = ros::Time();
      marker.ns = "";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.01;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      vis_pub.publish( marker );

    }

    void publishTF(double x, double y, double z){
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(x, y, z) );
      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "left_hand_camera", "circle"));
    }
    

    
}

void HoughDetection(const Mat& src_gray, const Mat& src_display, int cannyThreshold, int accumulatorThreshold, int radiusThreshold){
    // will hold the results of the detection
    std::vector<Vec3f> circles;
    // runs the actual detection
    HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, cannyThreshold, accumulatorThreshold, 0, 0 );

    // clone the colour, input image for displaying purposes
    Mat display = src_display.clone();

    int x_min = 1000;
    int y_min = 1000;
    int d_min = 2000;

    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        
        if (radius < radiusThreshold  && radius > radiusThreshold-8){
          // circle center
          circle( display, center, 3, Scalar(0,255,0), -1, 8, 0 );
          // circle outline
          circle( display, center, radius, Scalar(0,0,255), 3, 8, 0 );
          // update the pick candidate with the min distance circle
          int d = abs(circles[i][0] - 320) + abs(circles[i][1] - 200);
          if( d < d_min){
            d_min = d;
            x_min = circles[i][0];
            y_min = circles[i][1];
          }
          
        }
    }

    // find the 3d position for the pick candidate
    
    if( d_min < 2000){
      w = (double)height / 100.0;
      v(0) = cvRound(x_min)*w;
      v(1) = cvRound(y_min)*w;
      v(2) = w;
      VectorXd p = m * v;
      //publishMarker(p(0), p(1), p(2));
      circle( display, Point(x_min , y_min), 38, Scalar(255,0,0), 3, 8, 0 );
      // offset ajustment for the distance between gripper center and camera center
      double left_offset = 0.01;
      double right_offset = 0.03;
      publishTF(p(0) - left_offset , p(1), p(2));
      // publishTF(p(0) - right_offset, p(1), p(2));
    }

    // display the center of the image
    // circle( display, Point(320, 200), 3, Scalar(255,0,0), -1, 8, 0 );

    // shows the results
    imshow( windowName, display);
}



void imageCallback(const sensor_msgs::Image& msgs_image) {


  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msgs_image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  Mat src, src_gray;  

  // Read the image
  src = cv_ptr->image;

  // Convert it to gray
  cvtColor( src, src_gray, COLOR_BGR2GRAY );

  // Reduce the noise so we avoid false circle detection
  GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );

  // those paramaters cannot be =0
  // so we must check here
  cannyThreshold = std::max(cannyThreshold, 1);
  accumulatorThreshold = std::max(accumulatorThreshold, 1);

  //runs the detection, and update the display
  HoughDetection(src_gray, src, cannyThreshold, accumulatorThreshold, radiusThreshold);

  cv::waitKey(10);

}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "detect_circles");
    ros::NodeHandle nh;

    // matrix calculation

    double xp = 100, yp = 200;
    //Vector3d v(xp*w,yp*w,w);
    m(0,0) = 2.45000797e-03;
    m(0,1) = -1.35525272e-18;
    m(0,2) = -7.73883931e-01;
    m(1,0) = 4.87890978e-19;
    m(1,1) = 2.45000797e-03;
    m(1,2) = -4.50953478e-01;
    m(2,0) = 2.16840434e-19;
    m(2,1) = 1.30104261e-18;
    m(2,2) = 1;
    m(3,0) = 0;
    m(3,1) = 0;
    m(3,2) = 0;
    v(2) = w;

    // create the main window, and attach the trackbars
    namedWindow( windowName, WINDOW_AUTOSIZE );
    createTrackbar(cannyThresholdTrackbarName, windowName, &cannyThreshold,maxCannyThreshold);
    createTrackbar(accumulatorThresholdTrackbarName, windowName, &accumulatorThreshold, maxAccumulatorThreshold);
    createTrackbar(radiusThresholdTrackbarName, windowName, &radiusThreshold, maxRadiusThreshold);
    createTrackbar("Camera Height", windowName, &height, 100);

    // subscribe to the image topic
    ros::Subscriber sub = nh.subscribe("/cameras/left_hand_camera/image", 1, imageCallback);

    // marker publisher
    vis_pub = nh.advertise<visualization_msgs::Marker>( "detected_circles", 0 );

    

    ros::spin();

    return 0;
}
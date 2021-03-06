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
#include <std_msgs/String.h>
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
    const int cannyThresholdInitialValue = 50; // for table 90
    const int accumulatorThresholdInitialValue = 41; // for table 41
    const int radiusThresholdInitialValue = 34; // for table 38
    const int heightInitialValue = 34; // for table 33
    const int intensityInitialValue = 80;
    const int maxAccumulatorThreshold = 200;
    const int maxCannyThreshold = 255;
    const int maxRadiusThreshold = 100;
    const int minRadiusThreshold = 20;
    const int maxIntensityThreshold = 255;
    const int minIntensityThreshold = 0;

    // path of the package
    const std::string path = ".";

    // whether process
    bool process_bool = false;


    // declare and initialize both parameters that are subjects to change
    int cannyThreshold = cannyThresholdInitialValue;
    int accumulatorThreshold = accumulatorThresholdInitialValue;
    int radiusThreshold = radiusThresholdInitialValue;
    int height = heightInitialValue;
    int intenstiyThreshold = intensityInitialValue;

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

    void publishTF(double x, double y, double z, std::string tf_target){
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(x, y, z) );
      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "left_hand_camera", tf_target));
    }
    

    
}

void compareHistogram(Mat& test){
    Mat hsv_test;
    Mat src_lemon, hsv_lemon;
    Mat src_lime, hsv_lime;

    // load image
    src_lemon = imread("./lemon.jpg");
    src_lime = imread("./lime.jpg");

    // Conver to HSV
    cvtColor( test, hsv_test, COLOR_BGR2HSV );
    cvtColor( src_lemon, hsv_lemon, COLOR_BGR2HSV );
    cvtColor( src_lime, hsv_lime, COLOR_BGR2HSV );

    /// Using 50 bins for hue and 60 for saturation
    int h_bins = 50; int s_bins = 60;
    int histSize[] = { h_bins, s_bins };

    // hue varies from 0 to 179, saturation from 0 to 255
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, s_ranges };

    // Use the o-th and 1-st channels
    int channels[] = { 0, 1 };

    /// Histograms
    MatND hist_test;
    MatND hist_lemon;
    MatND hist_lime;

    /// Calculate the histograms for the HSV images
    calcHist( &hsv_test, 1, channels, Mat(), hist_test, 2, histSize, ranges, true, false );
    normalize( hist_test, hist_test, 0, 1, NORM_MINMAX, -1, Mat() );

    calcHist( &hsv_lemon, 1, channels, Mat(), hist_lemon, 2, histSize, ranges, true, false );
    normalize( hist_lemon, hist_lemon, 0, 1, NORM_MINMAX, -1, Mat() );

    calcHist( &hsv_lime, 1, channels, Mat(), hist_lime, 2, histSize, ranges, true, false );
    normalize( hist_lime, hist_lime, 0, 1, NORM_MINMAX, -1, Mat() );

    int compare_method = 0;

    double test_test = compareHist( hist_test, hist_test, compare_method );
    double test_lemon = compareHist( hist_test, hist_lemon, compare_method );
    double test_lime = compareHist( hist_test, hist_lime, compare_method );

    ROS_INFO( " Method 0 Perfect, test_test, test_lemon, test_lime : %f, %f, %f \n",  test_test, test_lemon , test_lime);
}

Mat HoughDetection(const Mat& src_gray, const Mat& src_display, int cannyThreshold, int accumulatorThreshold, int radiusThreshold){
    // will hold the results of the detection
    std::vector<Vec3f> circles;
    // runs the actual detection
    HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, cannyThreshold, accumulatorThreshold, 0, 0 );

    // clone the colour, input image for displaying purposes
    Mat display = src_display.clone();

    int x_min_lemon = 1000;
    int y_min_lemon = 1000;
    int d_min_lemon = 2000;
    int x_min_lime = 1000;
    int y_min_lime = 1000;
    int d_min_lime = 2000;

    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        
        if (radius < radiusThreshold  && radius > radiusThreshold-8){

          int intensity = src_gray.at<uchar>(circles[i][1], circles[i][0] );
          ROS_INFO("intensity is: %d", intensity);
          // lemon case
          if(intensity > intenstiyThreshold){
              // circle center
              circle( display, center, 3, Scalar(0,255,0), -1, 8, 0 );
              // circle outline
              circle( display, center, radius, Scalar(0,0,255), 3, 8, 0 );
              // update the pick candidate with the min distance circle
              // 320 and 200 are the offset pixel obtained in the camera calibration info
              int d = abs(circles[i][0] - 320) + abs(circles[i][1] - 200);
              if( d < d_min_lemon){
                d_min_lemon = d;
                x_min_lemon = circles[i][0];
                y_min_lemon = circles[i][1];
              }
          }else{ // lime case
              // circle center
              circle( display, center, 3, Scalar(0,255,0), -1, 8, 0 );
              // circle outline
              circle( display, center, radius, Scalar(0,255,0), 3, 8, 0 );
              // update the pick candidate with the min distance circle
              int d = abs(circles[i][0] - 320) + abs(circles[i][1] - 200);
              if( d < d_min_lime){
                d_min_lime = d;
                x_min_lime = circles[i][0];
                y_min_lime = circles[i][1];
              }
          
          }
        }
          
    }

    // find the 3d position for the pick candidate

    // create a mask
    Mat mask = Mat::zeros(400, 640, CV_8U); // all 0
    
    // lemon tf 
    if( d_min_lemon < 2000){
      w = (double)height / 100.0;
      v(0) = cvRound(x_min_lemon)*w;
      v(1) = cvRound(y_min_lemon)*w;
      v(2) = w;
      VectorXd p = m * v;
      //publishMarker(p(0), p(1), p(2));
      // circle( display, Point(x_min_lemon , y_min_lemon), 38, Scalar(255,0,0), 3, 8, 0 );

      // draw circle on the mask
      // circle( mask, Point(x_min_lemon , y_min_lemon), 30, Scalar(255, 255, 255), -1, 8, 0 );

      // offset ajustment for the distance between gripper center and camera center
      double left_offset = 0.00;
      double right_offset = 0.03;
      publishTF(p(0) - left_offset , p(1), p(2), "lemon");
      // publishTF(p(0) - right_offset, p(1), p(2));
    }

    // lime tf

    if( d_min_lime < 2000){
      w = (double)height / 100.0;
      v(0) = cvRound(x_min_lime)*w;
      v(1) = cvRound(y_min_lime)*w;
      v(2) = w;
      VectorXd p = m * v;
      //publishMarker(p(0), p(1), p(2));
      // circle( display, Point(x_min_lime , y_min_lime), 38, Scalar(255,0,0), 3, 8, 0 );

      // draw circle on the mask
      // circle( mask, Point(x_min_lime , y_min_lime), 30, Scalar(255, 255, 255), -1, 8, 0 );

      // offset ajustment for the distance between gripper center and camera center
      double left_offset = 0.00;
      double right_offset = 0.03;
      publishTF(p(0) - left_offset , p(1), p(2), "lime");
      // publishTF(p(0) - right_offset, p(1), p(2));
    }

    // display the center of the image
    // circle( display, Point(320, 200), 3, Scalar(255,0,0), -1, 8, 0 );
    
    
    Mat res;
    src_display.copyTo(res, mask);
    // compareHistogram(res);

    // shows the results
    imshow( windowName, display);
    return display;
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
  Mat res;
  if (process_bool) res = HoughDetection(src_gray, src, cannyThreshold, accumulatorThreshold, radiusThreshold);
  else{
    res = src;
    imshow( windowName, src);
  }

  cv::waitKey(10);

  // publish the processed image

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", res).toImageMsg();

  ros::Rate loop_rate(5);
  vis_pub.publish(msg);
/*  ros::spinOnce();
  loop_rate.sleep();*/

  

}

void processCallback(const std_msgs::String::ConstPtr& msg){
  if( std::strcmp(msg->data.c_str(),"start")==0 ){
    process_bool = true;
    ROS_INFO("circle detection starts");
  }
  if(std::strcmp(msg->data.c_str(),"stop")==0){
    process_bool = false;
    ROS_INFO("circle detection stops");
  }
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "detect_circles");
    ros::NodeHandle nh;

    // matrix calculation

    double xp = 100, yp = 200;
    /*
     rostopic echo -n 1 /cameras/left_hand_camera/camera_info
     
     header: 
       seq: 5759
       stamp: 
         secs: 1464196158
         nsecs: 882176000
       frame_id: /left_hand_camera
     height: 400
     width: 640
     distortion_model: plumb_bob
     D: [0.0116776045518, -0.0407996905853, -0.00110308187907, -5.38190873443e-05, 0.00866092069293]
     K: [408.161937671, 0.0, 635.869964945, 0.0, 408.161937671, 384.06204543, 0.0, 0.0, 1.0]
     R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
     P: [408.161937671, 0.0, 315.869964945, 0.0, 0.0, 408.161937671, 184.06204543, 0.0, 0.0, 0.0, 1.0, 0.0]
     binning_x: 1
     binning_y: 1
     roi: 
       x_offset: 320
       y_offset: 200
       height: 400
       width: 640
       do_rectify: False
     ---


     import numpy as np
     a = np.asarray([[408.161937671, 0.0, 315.869964945, 0.0, 0.0, 408.161937671, 184.06204543, 0.0, 0.0, 0.0, 1.0, 0.0]])
     b = a.reshape((3,4))
     m = np.linalg.pinv(b)
    */
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
    createTrackbar("Intensity Threshold", windowName, &intenstiyThreshold, 255);

    // subscribe to the image topic
    ros::Subscriber sub = nh.subscribe("/cameras/left_hand_camera/image", 1, imageCallback);
    // subscribe to process command
    ros::Subscriber sub_process = nh.subscribe("circle_detection_process", 1, processCallback);

    // marker publisher
    vis_pub = nh.advertise<sensor_msgs::Image>( "rvs_lemon_detection", 0 );

    

    ros::spin();

    return 0;
}
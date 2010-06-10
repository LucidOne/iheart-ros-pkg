/*
 *  OpenCV Demo for ROS
 *  Copyright (C) 2010, I Heart Robotics
 *  I Heart Robotics <iheartrobotics@gmail.com>
 *  http://www.iheartrobotics.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>

// ROS/OpenCV HSV Demo
// Based on http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

class Demo
{

protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  cv::Mat img_in_;
  cv::Mat img_hsv_;
  cv::Mat img_hue_;
  cv::Mat img_sat_;
  cv::Mat img_bin_;
  cv::Mat img_out_;
  IplImage *cv_input_;

public:

    Demo (ros::NodeHandle & nh):nh_ (nh), it_ (nh_)
  {
    // Listen for image messages on a topic and setup callback
    image_sub_ = it_.subscribe ("/usb_cam/image_raw", 1, &Demo::imageCallback, this);
    // Open HighGUI Window
    cv::namedWindow ("input", 1);
    cv::namedWindow ("binary image", 1);
    cv::namedWindow ("segmented output", 1);
  }

  void imageCallback (const sensor_msgs::ImageConstPtr & msg_ptr)
  {
    // Convert ROS Imput Image Message to IplImage
    try
    {
      cv_input_ = bridge_.imgMsgToCv (msg_ptr, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException error)
    {
      ROS_ERROR ("CvBridge Input Error");
    }

    // Convert IplImage to cv::Mat
    img_in_ = cv::Mat (cv_input_).clone ();
    // output = input
    img_out_ = img_in_.clone ();
    // Convert Input image from BGR to HSV
    cv::cvtColor (img_in_, img_hsv_, CV_BGR2HSV);
    // Zero Matrices
    img_hue_ = cv::Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
    img_sat_ = cv::Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
    img_bin_ = cv::Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
    // HSV Channel 0 -> img_hue_ & HSV Channel 1 -> img_sat_
    int from_to[] = { 0,0, 1,1};
    cv::Mat img_split[] = { img_hue_, img_sat_};
    cv::mixChannels(&img_hsv_, 3,img_split,2,from_to,2);

    for(int i = 0; i < img_out_.rows; i++)
    {
      for(int j = 0; j < img_out_.cols; j++)
      {
        // The output pixel is white if the input pixel
        // hue is orange and saturation is reasonable
        if(img_hue_.at<uchar>(i,j) > 4 &&
           img_hue_.at<uchar>(i,j) < 28 &&
           img_sat_.at<uchar>(i,j) > 96) {
          img_bin_.at<uchar>(i,j) = 255;
        } else {
          img_bin_.at<uchar>(i,j) = 0;
          // Clear pixel blue output channel
          img_out_.at<uchar>(i,j*3+0) = 0;
          // Clear pixel green output channel
          img_out_.at<uchar>(i,j*3+1) = 0;
          // Clear pixel red output channel
          img_out_.at<uchar>(i,j*3+2) = 0;
        }
      }
    }

    // Display Input image
    cv::imshow ("input", img_in_);
    // Display Binary Image
    cv::imshow ("binary image", img_bin_);
    // Display segmented image
    cv::imshow ("segmented output", img_out_);

    // Needed to  keep the HighGUI window open
    cv::waitKey (3);
  }

};


int
main (int argc, char **argv)
{
  // Initialize ROS Node
  ros::init (argc, argv, "ihr_demo1");
  // Start node and create a Node Handle
  ros::NodeHandle nh;
  // Instaniate Demo Object
  Demo d (nh);
  // Spin ...
  ros::spin ();
  // ... until done
  return 0;
}

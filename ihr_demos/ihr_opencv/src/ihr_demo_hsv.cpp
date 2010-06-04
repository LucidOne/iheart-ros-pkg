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
  image_transport::Publisher image_pub_;
  cv::Mat img_in_;
  cv::Mat img_hsv_;
  IplImage *cv_input_;
  IplImage cv_output_;

public:

    Demo (ros::NodeHandle & nh):nh_ (nh), it_ (nh_)
  {
    // Advertise image messages to a topic
    image_pub_ = it_.advertise ("/demo/output_image", 1);
    // Listen for image messages on a topic and setup callback
    image_sub_ = it_.subscribe ("/usb_cam/image_raw", 1, &Demo::imageCallback, this);
    // Open HighGUI Window
    cv::namedWindow ("hsv", 1);
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
    // Convert Input image from BGR to HSV
    cv::cvtColor (img_in_, img_hsv_, CV_BGR2HSV);
    // Display HSV Image in HighGUI window
    cv::imshow ("hsv", img_hsv_);

    // Needed to  keep the HighGUI window open
    cv::waitKey (3);

    // Convert cv::Mat to IplImage
    cv_output_ = img_hsv_;
    // Convert IplImage to ROS Output Image Message and Publish
    try
    {
      image_pub_.publish (bridge_.cvToImgMsg (&cv_output_, "bgr8"));
    }
    catch (sensor_msgs::CvBridgeException error)
    {
      ROS_ERROR ("CvBridge Output error");
    }
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

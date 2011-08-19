/*
 *  Teleoperation Package for WowWee Rovio
 *  Copyright (C) 2010 I Heart Robotics <iheartrobotics@gmail.com>
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
#include <std_msgs/String.h>
#include <joy/Joy.h>
#include <geometry_msgs/Twist.h>
#include <rovio_common/Head.h>

class TeleopRovio
{
public:
  TeleopRovio(void);

private:
  void joyCallback(const joy::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  geometry_msgs::Twist cmd;

  ros::ServiceClient head_client_;
  rovio_common::Head head_srv_;
};

TeleopRovio::TeleopRovio(void)
{
  // Right hand co-ordinate system
  // X+ is Forward
  // Y+ is Right
  // Z+ is Down
  // Positive rotation around the Z axis
  //   makes the robot rotate right

  cmd.linear.x=0;
  cmd.linear.y=0;
  cmd.linear.z=0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 0;
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  joy_sub_ = nh_.subscribe("joy", 10, &TeleopRovio::joyCallback, this);
  head_client_ = nh_.serviceClient<rovio_common::Head>("head_position");
}

void TeleopRovio::joyCallback(const joy::Joy::ConstPtr& joy)
{
  // FIXME: Make axes & buttons configuarable via parameters
  cmd.linear.x = joy->axes[1];
  cmd.linear.y = joy->axes[0] * -1;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = joy->axes[2] * -1;
  vel_pub_.publish(cmd);

  // FIXME: Add E-Stop
  if (joy->buttons[1] == 1)
  {
    head_srv_.request.position = rovio_common::Head::Request::DOWN;
    if (head_client_.call(head_srv_))
    {
      // FIXME: Actually return status here
      ROS_INFO("Head Status: %d", (int)head_srv_.response.status);
    }
    else
    {
      ROS_ERROR("Failed to call service head_position");
    }
  }
  else if (joy->buttons[2] == 1)
  {
    head_srv_.request.position = rovio_common::Head::Request::MID;
    if (head_client_.call(head_srv_))
    {
      ROS_INFO("Head Status: %d", (int)head_srv_.response.status);
    }
    else
    {
      ROS_ERROR("Failed to call service head_position");
    }
  }
  else if (joy->buttons[3] == 1)
  {
    head_srv_.request.position = rovio_common::Head::Request::UP;
    if (head_client_.call(head_srv_))
    {
      ROS_INFO("Head Status: %d", (int)head_srv_.response.status);
    }
    else
    {
      ROS_ERROR("Failed to call service head_position");
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rovio_teleop");
  TeleopRovio teleop_rovio;
  ros::spin();
}

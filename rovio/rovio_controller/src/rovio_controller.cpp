/*    
 *  Controller Package for WowWee Rovio
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

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <ros/ros.h>
#include <resource_retriever/retriever.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <rovio_common/Head.h>

class RovioController
{
public:
  RovioController(void);
  void update(void);

private:
  void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd);
  bool headCallback(rovio_common::Head::Request  &req,
         rovio_common::Head::Response &res );
  bool movementCommand(int action,int drive, int speed);
  
  ros::NodeHandle nh_;

  geometry_msgs::Twist vel;
  std::string rovio_hostname;
  std::string rovio_username;
  std::string rovio_password;
  int drive;
  int speed;
  int rotate;
  rovio_common::Head head;
  ros::Subscriber vel_sub_;
  ros::Publisher com_pub_;
  ros::ServiceServer head_srv_;

  resource_retriever::Retriever r;
  resource_retriever::MemoryResource resource;
};

RovioController::RovioController(void)
{
  vel.linear.x=0;
  vel.linear.y=0;
  vel.linear.z=0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = 0;
  ros::NodeHandle nh_param("~");
  if (!nh_param.hasParam("rovio_hostname"))
  {
    ROS_ERROR("Username Required");
    exit(0);
  }
  if (!nh_param.hasParam("rovio_password"))
  {
    ROS_ERROR("Password Required");
    exit(0);
  }
  if (!nh_param.hasParam("rovio_hostname"))
  {
    rovio_hostname = "192.168.10.18";
  } else {
    nh_param.getParam("rovio_hostname", rovio_hostname);
  }
  nh_param.getParam("rovio_username", rovio_username);
  nh_param.getParam("rovio_password", rovio_password);
  vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &RovioController::cmdCallback, this);
  head_srv_ = nh_.advertiseService("head_position", &RovioController::headCallback,this);
}

bool RovioController::movementCommand(int action,int drive, int speed)
{
  char url[256];
  try
  {
    //resource = r.get("package://resource_retriever/test/test.txt"); 
    sprintf(url,"http://%s:%s@%s/rev.cgi?Cmd=nav&action=%d&drive=%d&speed=%d",
      rovio_username.c_str(),rovio_password.c_str(),rovio_hostname.c_str(),action,drive,speed);
    resource = r.get(url);
  }
  catch (resource_retriever::Exception& e)
  {
    ROS_ERROR("Failed to send command: %s", e.what());
    return 1;
  }
}

bool RovioController::headCallback(rovio_common::Head::Request  &req,
         rovio_common::Head::Response &res )
{
  // Note: The head can be positioned at any point by sending
  // a movement command followed by a stop
  switch (req.position)
  {
    case rovio_common::Head::Request::DOWN:
      movementCommand(18,12,0);
      break;
    case rovio_common::Head::Request::MID:
      movementCommand(18,13,0);
      break;
    case rovio_common::Head::Request::UP:
      movementCommand(18,11,0);
      break;
  }
  res.status = 1;
  return true;
}

void RovioController::update(void)
{
  if (drive != 0) {
    movementCommand(18,drive,speed);
  }
  if (rotate != 0) {
    if (rotate > 0) {
      movementCommand(18,6,rotate);
    } else {
      movementCommand(18,5,-1 * rotate);
    }
  }
}

void RovioController::cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  // FIXME: Calibrate and rescale velocities so that 1 = 1 m/s
  // rotation should be in radians/second
  double angle;
  vel.linear.x = (cmd->linear.x > 0.070711 || cmd->linear.x < 0.070711) ? cmd->linear.x : 0;
  vel.linear.y = (cmd->linear.y > 0.070711 || cmd->linear.y < 0.070711) ? cmd->linear.y : 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = (cmd->angular.z > 0.1 || cmd->angular.z < -0.1) ? cmd->angular.z : 0;
  // Angular Movement
  // Note: The Rovio API sets 10 as the slowest and 1 as the fastest
  if (vel.angular.z == 0) { 
    rotate = 0;
  } else {
    rotate = 10 / int(vel.angular.z * 10);
  }

  // Linear Movement
  // FIXME: This is a mess, but the Rovio API limits solutions
  speed = int((pow(vel.linear.x,2) + pow(vel.linear.y,2))*10);
  speed = (speed > 10) ? 10 : speed;
  speed = (speed < 1) ? 1 : speed;
  speed = 10 / speed;
  if (vel.linear.x == 0 && vel.linear.y < 0)
  {
    drive = 3;
    angle = 3*M_PI/2;
  }
  else if (vel.linear.x == 0 && vel.linear.y > 0)
  {
    drive = 4;
    angle = M_PI/2;
  }
  else
  {
    angle = atan2(vel.linear.x,vel.linear.y);
    if (vel.linear.x > 0 && angle <= 5*M_PI/8 && angle >= 3*M_PI/8)
    {
      drive = 1;
    }
    else if (vel.linear.x > 0 && vel.linear.y > 0 && angle < 3*M_PI/8 && angle >= M_PI/8)
    {
      drive = 8;
    }
    else if (vel.linear.y > 0 && (angle >= -1*M_PI/8 || angle >= 3*M_PI/8))
    {
      drive = 4;
    }
    else if (vel.linear.x < 0 && vel.linear.y > 0 && angle < -1*M_PI/8 && angle >= -3*M_PI/8)
    {
      drive = 10;
    }
    else if (vel.linear.x < 0 && angle <= -3*M_PI/8 && angle >= -5*M_PI/8)
    {
      drive = 2;
    }
    else if (vel.linear.x < 0 && vel.linear.y < 0 && angle <= -5*M_PI/8 && angle >= -7*M_PI/8)
    {
      drive = 9;
    }
    else if (vel.linear.y < 0 && (angle < -7*M_PI/8 || angle >= 7*M_PI/8))
    {
      drive = 3;
    }
    else if (vel.linear.x > 0 && vel.linear.y < 0 && angle < 7*M_PI/8 && angle >= 5*M_PI/8)
    {
      drive = 7;
    }
    else
    {
      drive = 0;
    }
  }
  
}

int main(int argc, char** argv)
{
  ROS_INFO("Starting Rovio Controller");
  ros::init(argc, argv, "rovio_controller");
  RovioController rovio_controller;
  ros::Rate r(5); // Update frequency in Hz
  // FIXME: This is WRONG, catch shutdown signals here.
  while (1)
  {
    rovio_controller.update();
    ros::spinOnce();
    r.sleep();
  }
}

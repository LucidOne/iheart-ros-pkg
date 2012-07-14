#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <turtlebot_button/Buttons.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

const int NBUTTONS = 5;

class TurtlebotButtonOp
{
public:
	TurtlebotButtonOp();

private:
	void buttonCallback(const turtlebot_button::Buttons::ConstPtr& button);
	void estopCallback(const std_msgs::Bool::ConstPtr& estop);
	void publish();

	ros::NodeHandle ph, nh;
	ros::Publisher twist_pub;
	ros::Subscriber button_sub;
	ros::Subscriber estop_sub;
	
	uint8_t last_button_state[NBUTTONS];
	uint8_t estop_state;

	geometry_msgs::Twist last_published;
	boost::mutex publish_mutex;
	ros::Timer timer;
};

TurtlebotButtonOp::TurtlebotButtonOp() {
	twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	button_sub = nh.subscribe<turtlebot_button::Buttons>("buttons", 100, &TurtlebotButtonOp::buttonCallback, this);
	estop_sub = nh.subscribe<std_msgs::Bool>("estop", 1, &TurtlebotButtonOp::estopCallback, this);

	timer = nh.createTimer(ros::Duration(0.5), boost::bind(&TurtlebotButtonOp::publish, this));
}

void TurtlebotButtonOp::buttonCallback(const turtlebot_button::Buttons::ConstPtr& button) {
	geometry_msgs::Twist vel;
	uint8_t i;
	
	for (i = 0; i < button->button_state.size(); i++) {
		if ((last_button_state[i] != button->button_state[i]) && button->button_state[i]) {
			switch (i) {
				case 0:
					vel.angular.z = 1;
					
					break;
				case 1:
					vel.linear.x = 1;
					
					break;
				case 2:
					vel.linear.x = -1;
					
					break;
				case 3:
					vel.angular.z = -1;					

					break;
				case 4:
					
					
					break;					
				default:
					
					break;
			}
		}
	}

	last_published = vel;

	for (uint8_t i = 0; i < button->button_state.size(); i++) {
		last_button_state[i] = button->button_state[i];
	}
}

void TurtlebotButtonOp::estopCallback(const std_msgs::Bool::ConstPtr& estop) {
	estop_state = estop->data;
}

void TurtlebotButtonOp::publish() {
	boost::mutex::scoped_lock lock(publish_mutex);
	
	if (estop_state) {
		twist_pub.publish(last_published);
	} else {
		
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "turtlebot_button_op");
	TurtlebotButtonOp turtlebot_button_op;

	ros::spin();
}

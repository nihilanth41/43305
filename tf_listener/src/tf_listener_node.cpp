#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <leap_motion/leapros.h>
#include <std_msgs/Header.h>
#include "Puma_OP.h"

#define TOPIC_SIZE 1000

int main(int argc, char **argv {
	ros::init(argc, argv, "tf_listener_node");
	ros::NodeHandle nh;

	tf::TransformListener listener;
	ros::Rate rate(10);
	while(nh.ok()) {
		tf::StampedTransform t1, t2, t3;
		try { 
			listener.lookupTransform("origin", "robot_base", ros::Time(0), t1);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		try { 
			listener.lookupTransform("origin", "leap_motion", ros::Time(0), t2);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		try { 
			listener.lookupTransform("leap_motion", "hand", ros::Time(0), t3);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		// hand wrt rb == inverse(robot base wrt origin) x lm wrt origin x hand wrt lm
		tf::Transform t4 = t1.inverseTimes(t2*t3);
		tf::Vector3 pos = t4.getOrigin();
		tf::Quaternion quat = t4.getRotation();
		
		/* PUMA API */
		//POSITION p;
		//p.X = 

		rate.sleep();
	}
	return 0;
}


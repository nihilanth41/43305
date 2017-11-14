#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <leap_motion/leapros.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>
#include "Puma_OP.h"

#define TOPIC_SIZE 1000

//TODO: Add subscriber/callback to get hand position(s) from /leapmotion/data topic
// Can do broadcast *in* callback or main loop.
void leapmotion_callback(const leap_motion::leapros &msg) {
	std_msgs::Header header = msg.header;	// uint32 seq, time stamp, string frame_id 
	tf::Vector3 palmpos(msg.palmpos.x, msg.palmpos.y, msg.palmpos.z);
	tf::Vector3 direction(msg.direction.x, msg.direction.y, msg.direction.z);  // Direction vector points 'forward' out of the hand (orientation of fingers)
	tf::Vector3 normal(msg.normal.x, msg.normal.y, msg.normal.z);	// Normal vector points perpendicularly out of the hand (orientation of the palm)

	tf::Vector3 N,O,A;
	// palmNormal is hand Z axis <-> A-Axis
	// palmDirection is hand Y axis <-> O-Axis
	A = normal; // Z
	O = direction; // Y
	N = O.cross(A); // X = YxZ
	tf::Matrix3x3 hand_orientation_m(N.x, N.y, N.z, O.x, O.y, O.z, A.x, A.y, A.z);
	tf::Transform t3(hand_orientation_m.inverse(), palmpos);
	//ROS_INFO("Palm x:%lf y:%lf z:%lf", msg.palmpos.x, msg.palmpos.y, msg.palmpos.z);
	//br.sendTransform(tf::StampedTransform(t3, header.stamp, "leap_motion", "hand");
	br.sendTransform(tf::StampedTransform(t3, ros::Time::now(), "leap_motion", "hand"));
}

//http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
int main(int argc, char **argv) {
	ros::init(argc, argv, "tf_broadcaster_node");
	ros::NodeHandle nh;

	static tf::Transform t1, t2;
	static tf::Quaternion q1, q2;
	static tf::TransformBroadcaster br;

	// Robot base w.r.t origin
	// Origin of the robot base w.r.t origin is same x,y as origin with z=0.325m=325mm
	t1.setOrigin(tf::Vector3(0.0, 0.0, 0.325));
	q1.setRPY(0, 0, 0);	// We're setting origin so it's oriented same as robot base.
	t1.setRotation(q1);	// Find quaternion for this frame

	// Position of leap motion w.r.t robot base.
	t2.setOrigin(tf::Vector3(-0.1, 0.4, 0.01)); // -100cm, +400cm, +1cm, xyz
	q2.setRPY(90, 0, -90); // Orientation of Leapmotion w.r.t robot base. R=90 to match Z-axis, Y=-90 to match x-axis
	t2.setRotation(q2);

	// leapmotion -> hand -- from leapmotion/data
	br.sendTransform(tf::StampedTransform(t1, ros::Time::now(), "origin", "robot_base")); // Position of robot base  w.r.t origin
	br.sendTransform(tf::StampedTransform(t2, ros::Time::now(), "origin", "leap_motion")); // Position of Leapmotion base w.r.t origin
	ros::Subscriber sub=nh.subscribe("leapmotion/data", TOPIC_SIZE, &leapmotion_callback);

	return 0;
}

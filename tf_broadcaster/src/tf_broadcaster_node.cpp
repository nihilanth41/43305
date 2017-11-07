#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//TODO: Add subscriber/callback to get hand position(s) from /leapmotion/data topic
// Can do broadcast *in* callback or main loop.
// 

//http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
int main(int argc, char **argv) {
	ros::init(argc, argv, "tf_broadcaster");
	ros::NodeHandle nh;

	// Listen to topic /leapmotion/data (type leap_motion/leapros)
	// rosmsg show leapros
	// Find palmpos 
	// palmNormal is hand Z axis <-> A-Axis
	// palmDirection is hand X-axis <-> N-Axis
	// Need to find unit vector for O axis as: ZcrossX
	// XYZ of hand comes from Point palmpos

	static tf::TransformBroadcaster br;

	tf::Transform t1, t2, t3;
	tf::Quaternion q1, q2, q3;

	// Robot base w.r.t origin
	// q.setRPY(0, 0, 0);
	// transform.setRotation(q);
	// Origin of the robot base w.r.t origin is same x,y as origin with z=0.325m=325mm
	t1.setOrigin(tf::Vector3(0.0, 0.0, 0.325));
	q1.setRPY(0, 0, 0);	// We're setting origin so it's oriented same as robot base.
	t1.setRotation(q1);

	// Position of leap motion w.r.t robot base.
	t2.setOrigin(tf::Vector3(-0.1, 0.4, 0.01)) // -100cm, +400cm, +1cm, xyz
	q2.setRPY(90, 0, -90); // Orientation of Leapmotion w.r.t robot base. R=90 to match Z-axis, Y=-90 to match x-axis
	t2.setRotation(q2);

	// leapmotion -> hand -- from leapmotion/data
	br.sendTransform(tf::StampedTransform(t1, ros::Time::now(), "origin", "robot_base")); // Position of robot base  w.r.t origin
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "origin", "leap_motion")); // Position of Leapmotion base w.r.t origin

	// Get the translation, rotation from the leapmotion topic /leapmotion/data
	// XYZ translation is from geometry_msgs/Point palmpos
	// Orientation opts: ypr, direction, normal -- which of these is easiest? 
	//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "leap_motion", "hand"));	    // Hands w.r.t origin
	return 0;
}

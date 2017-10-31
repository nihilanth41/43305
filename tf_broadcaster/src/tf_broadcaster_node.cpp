#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

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
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "origin"));
	return 0;
}

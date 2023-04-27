#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"

#include "first_project/Odom.h"
#include "first_project/reset_odom.h"
#include <stdlib.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <string>

// Global position values
// This way our service_cb is able to reset them
// get the parameter server values here.
static float posx = 0;
static float posy = 0;
static float orientx = 0;
static float orienty = 0;
static float orientz = 0;
static float orientw = 0;

static float cstx = 0;
static float csty = 0;
static float cstth = 0;

void bag_cb(const geometry_msgs::Quaternion::ConstPtr& msg){
	ROS_INFO("I heard [%f, %f, %f, %f]", msg->x, msg->y, msg->z, msg->w);
	return;
}

bool service_cb(first_project::reset_odom::Request &req, first_project::reset_odom::Response &res){
	ROS_INFO("/reset_odom service was called: Setting all odometry values to zero!");
	posx = 0;
	posy = 0;
	orientx = 0;
	orienty = 0;
	orientz = 0;
	orientw = 0;
	res.resetted = true;
	return true;
}

int main(int argc, char **argv){
    
	ros::init(argc, argv, "odom_node"); 
	ros::NodeHandle n;

	n.getParam("/starting_x", cstx);
	n.getParam("/starting_y", csty);
	n.getParam("/starting_th", cstth);
	ROS_INFO("Parameters [%f, %f, %f]", cstx, csty, cstth);

	// We publish the default Odometry message after calculation here
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 1000);

	// We publish the custom odometry message after calculation to this topic
	ros::Publisher cst_odom_pub = n.advertise<first_project::Odom>("custom_odometry", 1000);

	// Subscribe to the bags-topic to get the raw position data.
	ros::Subscriber bag_sub = n.subscribe("/speed_steer", 1000, bag_cb);

	// Advertise the reset service
	ros::ServiceServer service = n.advertiseService("reset_odom", service_cb);

	ros::Rate loop_rate(10);

	// Fill the values with our default parameters (normally 0)
	while (ros::ok()){
		// define messages that we will send
		nav_msgs::Odometry msg;
		first_project::Odom cst_msg;

		// For the time to work, we need to have a clock server(?)
		std::stringstream timestamp;
		ROS_INFO("%f", ros::Time::now().toSec());
		timestamp << std::to_string(ros::Time::now().toSec());

		cst_msg.x = cstx;
		cst_msg.y = csty;
		cst_msg.th = cstth;
		cst_msg.timestamp = timestamp.str();

		msg.pose.pose.position.x = posx;
		msg.pose.pose.position.y = posy;
		msg.pose.pose.orientation.x = orientx;
		msg.pose.pose.orientation.y = orienty;
		msg.pose.pose.orientation.z = orientz;
		msg.pose.pose.orientation.w = orientw;

		odom_pub.publish(msg);
		cst_odom_pub.publish(cst_msg);

		ros::spinOnce();

		loop_rate.sleep();
	}


  	return 0;
}


class tf_sub_pub {
	public:
		tf_sub_pub() {
			sub = n.subscribe('', 1000, &tf_sub_pub::callback, this);
		}
		void callback(const some::ting msg) {
			tf::Transform transform;
			transform.setOrigin();
			tf::Quaternion q;
			q.setRPY();
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform());
		}
	private:
		ros::NodeHandle n;
		tf::TransformBroadcaster br;
		ros::Subscriber sub;
};
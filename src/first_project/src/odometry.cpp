#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"

#include "first_project/Odom.h"
#include "first_project/reset_odom.h"
#include <stdlib.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <string>
#include <cmath>

// Global position values
// This way our service_cb is able to reset them
// get the parameter server values here.
static float orientx = 0;
static float orienty = 0;
static float orientz = 0;
static float orientw = 0;

static float cstx = 0;
static float csty = 0;
static float cstth = 0;

static int LOOP_RATE = 1000;

// The distance between front and rear wheels
const float d = 2.8;

ros::Time new_time;
ros::Time last_time;
ros::Duration tdelta_ROS;

void bag_cb(const geometry_msgs::Quaternion::ConstPtr& msg){
	// TODO: Get the first time before starting to calculate
	// Maybe call /clock topic somewhere before everything starts
	if (last_time.toSec() == 0) {
		last_time = ros::Time::now();
	}
	new_time = ros::Time::now();

	// On first call, last_time might be 0. After the first loop, all is good.
	double LTime = last_time.toSec();
	double NTime = new_time.toSec();

	// ROS_INFO("last_time : [%f]", LTime);
	// ROS_INFO("new_time : [%f]", NTime);

	tdelta_ROS = new_time - last_time;      
	float tdelta = tdelta_ROS.toSec();
	// ROS_INFO("TDELTA : [%f]", tdelta);

	last_time = new_time;

	float speed = msg->x;
	float steering_angle = msg->y;

	float R = 0;
	float angular_velocity = 0;

	R = d / tan(steering_angle); // distance from the center of the car to the ICC

	float omega = speed * tan(steering_angle) / d;
	ROS_INFO("OMEGA : [%f]", omega);

	if (abs(omega) < 0.001) { // TODO: Change to 0 + a small epsilon > abs(omega)
		ROS_INFO("OMEGA IS 0");
		cstx += speed * tdelta * cos(cstth + omega * tdelta / 2); 
		csty += speed * tdelta * sin(cstth + omega * tdelta / 2);
	} else {
	// NOTE: If we have omega ~ 0, we need to use the runge-kutta approximation instead
		cstx += (speed / omega) * (sin(cstth + omega * tdelta) - sin(cstth)); 
		csty += (speed / omega) * (cos(cstth + omega * tdelta) - cos(cstth));
		cstth += omega * tdelta; // angular velocity (angle / s) * time in seconds -> new angle
	}	

	ROS_INFO("CSTX : [%f]", cstx);
	ROS_INFO("CSTY : [%f]", csty);
	return;
}

bool service_cb(first_project::reset_odom::Request &req, first_project::reset_odom::Response &res){
	ROS_INFO("/reset_odom service was called: Setting all odometry values to zero!");
	orientx = 0;
	orienty = 0;
	orientz = 0;
	orientw = 0;
	cstx = 0;
	csty = 0;
	cstth = 0;

	res.resetted = true;
	return true;
}

int main(int argc, char **argv){
    
	ros::init(argc, argv, "odom_node"); 
	ros::NodeHandle n;
	tf::TransformBroadcaster br;

	n.getParam("/starting_x", cstx);
	n.getParam("/starting_y", csty);
	n.getParam("/starting_th", cstth);
	ROS_INFO("Parameters [%f, %f, %f]", cstx, csty, cstth);

	// We publish the default Odometry message after calculation here
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odometry", LOOP_RATE);

	// We publish the custom odometry message after calculation to this topic
	ros::Publisher cst_odom_pub = n.advertise<first_project::Odom>("custom_odometry", LOOP_RATE);

	// Subscribe to the bags-topic to get the raw position data.
	ros::Subscriber bag_sub = n.subscribe("/speed_steer", LOOP_RATE, bag_cb);

	// Advertise the reset service
	ros::ServiceServer service = n.advertiseService("reset_odom", service_cb);

	ros::Rate loop_rate(LOOP_RATE);

	while (ros::ok()){
		nav_msgs::Odometry msg;
		first_project::Odom cst_msg;

		//ROS_INFO("%f", ros::Time::now().toSec());
		cst_msg.x = cstx;
		cst_msg.y = csty; 
 		cst_msg.th = cstth;
		cst_msg.timestamp = std::to_string(ros::Time::now().toSec());

		// TODO: Correct TF implementation
		tf::Transform transform;
		transform.setOrigin( tf::Vector3( cstx, csty, 0) );
		tf::Quaternion q;
		q.setRPY(0, 0, cstth);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

		msg.pose.pose.position.x = cstx;
		msg.pose.pose.position.y = csty;
		msg.pose.pose.orientation.x = q.getX();
		msg.pose.pose.orientation.y = q.getY();
		msg.pose.pose.orientation.z = q.getZ();
		msg.pose.pose.orientation.w = q.getW();

		odom_pub.publish(msg);
		cst_odom_pub.publish(cst_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

  	return 0;
}

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
static float posx = 0;
static float posy = 0;
static float orientx = 0;
static float orienty = 0;
static float orientz = 0;
static float orientw = 0;

static float cstx = 0;
static float csty = 0;
static float cstth = 0;

float speed = 0;
float steering_angle = 0;

//float last_time = 0;
//float new_time = 0;
//float tdelta = 0;

ros::Time new_time;
ros::Time last_time;
ros::Duration tdelta_ROS;
std::stringstream tdelta;



void bag_cb(const geometry_msgs::Quaternion::ConstPtr& msg){
			//ROS_INFO("I heard [%f, %f, %f, %f]", msg->x, msg->y, msg->z, msg->w);
	// TODO: Get the first time before starting to calculate

	double LTime=last_time.toSec(); //
	ROS_INFO("lasttime : [%f]", LTime);      				//at this point we still have Lasttime =0 for the first loop
															//after its all good
	
			
	
	new_time = ros::Time::now(); 
	tdelta_ROS = new_time -last_time;      
	last_time=new_time;
	
	
	if (LTime>1)
	{
	
	double tdelta=tdelta_ROS.toSec();		//
	double NTime=new_time.toSec();         //
	ROS_INFO("newtime : [%f]", NTime);		//						//the lines below are for the computation of x,y and theta ...
	ROS_INFO("TDELTA : [%f]", tdelta);


	speed = msg->x;
	steering_angle = msg->y;

		
	//cstth = static_cast<int>(cstth + steering_angle*tdelta)   %		360;
	cstth = cstth + steering_angle*tdelta;
	cstx = cstx + speed *tdelta* sin(cstth*180/M_PI);   //I try to multiply by tdelta in the main loop     *tdelta 
	csty = csty + speed * tdelta * cos(cstth*180/M_PI);
	
	return;
	
	}
}



bool service_cb(first_project::reset_odom::Request &req, first_project::reset_odom::Response &res){
	ROS_INFO("/reset_odom service was called: Setting all odometry values to zero!");
	posx = 0;
	posy = 0;
	orientx = 0;
	orienty = 0;
	orientz = 0;
	orientw = 0;

	cstx = 0;
	csty = 0;
	cstth = 0;

	steering_angle = 0;

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

	ros::Rate loop_rate(1000);
  

	// Fill the values with our default parameters (normally 0)

	//We iniate our times Values and makeTdelta=0

	//	ros::Time timestamp= ros::Time::now();;
	//	ros::Time previous_timestamp= ros::Time::now();;
	//	ros::Duration tdelta_ROS = timestamp-previous_timestamp;
	//	std::stringstream tdelta;                                 //still a str we may have to switch for an int




	while (ros::ok()){
		// define messages that we will send
		nav_msgs::Odometry msg;
		first_project::Odom cst_msg;

		// For the time to work, we need to have a clock server(?)

		

/*ROS_INFO("%f", ros::Time::now().toSec());



		timestamp = ros::Time::now();
		tdelta_ROS = timestamp -previous_timestamp;      
		previous_timestamp=timestamp;
		ROS_INFO("Duration in seconds: %lf", tdelta_ROS.toSec());
		
		
		cst_msg.x = cst_msg.x + speed * tdelta * sin(steering_angle);    */



		cst_msg.x = cstx;
		cst_msg.y = csty;                           //so here i use csty that comes from    bag_cb   :     
 		cst_msg.th = cstth;										//Question is how can i update cst_msg.y with the value i will cupute here
		cst_msg.timestamp =   std::to_string(ros::Time::now().toSec()) ;
		

		msg.pose.pose.position.x = cstx;
		msg.pose.pose.position.y = csty;
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


// class tf_sub_pub {
// 	public:
// 		tf_sub_pub() {
// 			sub = n.subscribe('', 1000, &tf_sub_pub::callback, this);
// 		}
// 		void callback(const some::ting msg) {
// 			tf::Transform transform;
// 			transform.setOrigin();
// 			tf::Quaternion q;
// 			q.setRPY();
// 			transform.setRotation(q);
// 			br.sendTransform(tf::StampedTransform());
// 		}
// 	private:
// 		ros::NodeHandle n;
// 		tf::TransformBroadcaster br;
// 		ros::Subscriber sub;
// };
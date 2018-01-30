#include <ros/ros.h>
#include "ros_holodeck/ros_holodeck.h"

int main(int argc, char** argv){
	// start node
	ros::init(argc, argv, "ros_holodeck_node");


	holodeck::ROSHolodeck ros_holodeck;


	ros::spin();
	return 0;
}
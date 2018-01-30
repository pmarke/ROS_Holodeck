#include "holodeck_controller/controller.h"

int main(int argc, char** argv){
	// start node
	ros::init(argc, argv, "ros_holodeck_node");


	holodeck::Controller controller;


	ros::spin();
	return 0;
}
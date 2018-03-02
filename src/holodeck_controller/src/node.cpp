#include "frontend.h"

int main(int argc, char** argv){
	// start node
	ros::init(argc, argv, "holodeck_controller_node");


	holodeck::Frontend frontend;


	ros::spin();
	return 0;
}
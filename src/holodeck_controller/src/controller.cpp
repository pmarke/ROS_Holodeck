#include "holodeck_controller/controller.h"


namespace holodeck {


Controller::Controller() {

	// create private node handle
	ros::NodeHandle nh_("holodeck");

	command_sub_ = nh_.subscribe("joy", 10, &Controller::joy_callback, this);

	command_pub_ = nh_.advertise<ros_holodeck::command>("command",10);

}

void Controller::joy_callback(const sensor_msgs::Joy& msg) {


	// get joy commands
	if (msg.buttons[0])
		command_.reset = true;
	else
		command_.reset = false;

	if( abs(msg.axes[1]) > threshold_)
		altitude_ += msg.axes[1]*scale_;

	if( abs(msg.axes[3]) > threshold_)
		roll_ = msg.axes[3]*PI/4;
	else
		roll_ = 0;

	if( abs(msg.axes[0]) > threshold_)
		yaw_rate_ = msg.axes[0];
	else
		yaw_rate_ = 0;

	if( abs(msg.axes[4]) > threshold_)
		pitch_ = msg.axes[4]*PI/4;
	else
		pitch_ = 0;



	

	// clip commands if they exceed limits
	saturate(altitude_, 50, 0);

	publish_command();





}

void Controller::publish_command() {


	// pack values
	command_.pitch = -pitch_;
	command_.roll = -roll_;
	command_.yaw_rate = yaw_rate_;
	command_.altitude = -altitude_;

	// publish command
	command_pub_.publish(command_);

}

void Controller::saturate(float &command, float max_val, float min_val) {

	if (command < min_val)
		command = min_val;
	else if (command > max_val)
		command = max_val;

}





}
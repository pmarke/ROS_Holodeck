# pragma once

// ros
#include <ros/ros.h>
#include <ros_holodeck/command.h>
#include <sensor_msgs/Joy.h>
#include <stdlib.h>


namespace holodeck {

	#define PI 3.14159

	class Controller {

	public:

		Controller();

		// ros communication
		ros::Subscriber command_sub_;
		ros::Publisher command_pub_;

	private:

		ros_holodeck::command command_;
		sensor_msgs::Joy joy_;
		float scale_ = 0.5;

		float altitude_ =0;
		float pitch_ = 0;
		float roll_ = 0;
		float yaw_rate_ =0;

		// joystick must move beyond this region to be a valid command
		float threshold_ = 0.1;

		// get commands from joystick
		void joy_callback(const sensor_msgs::Joy& msg);

		// send commands to holodeck
		void publish_command();

		// make sure commands do not go beyond saturation level
		void saturate(float &command, float max_val, float min_val);

	};







}
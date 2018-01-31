#pragma once

// ros
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// ros holodeck
#include <ros_holodeck/state_srv.h>
#include <ros_holodeck/state.h>
#include <ros_holodeck/command.h>

// This project
#include "holodeck_controller/controller_manager/controller_OptFlow.h"
#include "holodeck_controller/controller_manager/controller_base.h"

// other libraries
#include <string>
#include <Eigen/Dense>
#include <iostream>
#include <ncurses.h>




namespace holodeck { 

// keyboard keys
	static const int key_w = 119;
	static const int key_a = 97;
	static const int key_s = 115;
	static const int key_d = 100;
	static const int key_r = 114;
	static const int key_c = 99;
	static const int key_q = 113;
	static const int key_space = 32;
	static const int key_arrow_up = 259;
	static const int key_arrow_down = 258;
	static const int key_arrow_left = 260;
	static const int key_arrow_right = 261;



	// This class sends commands to the ROS_Holodeck interface and receives
	// an image and state
	class Frontend {

	public:
		Frontend();

	private:
		// Ros
		ros::NodeHandle nh_;
		image_transport::Subscriber sub_video_;
		image_transport::Publisher pub_video_;
		ros::Publisher command_pub_;
		ros::ServiceClient srv_state_;


		// ROS_holodeck interface
		ros_holodeck::state state_;
		ros_holodeck::state_srv state_srv_;
		ros_holodeck::command command_;

		// Frames
		cv::Mat img_;
		cv::Mat grayImg_;
		cv::Mat alteredImg_;

		// If true, control using the keyboard
		bool use_keyboard_ = true;

		// Extenstions
		// ControllerManager controller_manager_;
		std::shared_ptr<ControllerBase> controller_base_;


		// Retrieve Holodeck Image
		void callback_sub_video(const sensor_msgs::ImageConstPtr& data);

		// publish commands to holodeck
		void publish_command();

		// get state information
		void service_state();

		// implement controller
		void implement_extensions();

		// get input from the keyboard
		void keyboard_input();

		void handle_key_input(int key_pressed);

	} ;







}
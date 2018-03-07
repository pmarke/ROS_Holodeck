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

// rqt_reconfigure
#include <holodeck_controller/frontendConfig.h>
#include <dynamic_reconfigure/server.h>

// This project
#include "controller_manager/controller_OptFlow.h"
#include "controller_manager/controller_base.h"
#include "visual_odometry/visual_odometry.h"
#include "feature_manager/feature_manager.h"

// Keyboard Input
#include "keyboard_input/keyboard.h"
#include "keyboard_input/instructions.h"

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

	static const float PI = 3.1415926535897;

// frame rate
	static const float frame_rate = 30;



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
		ros::Subscriber keyboard_sub_;
		ros::ServiceClient keyboard_srv_;


		// ROS_holodeck interface
		ros_holodeck::state state_;
		ros_holodeck::state_srv state_srv_;
		ros_holodeck::command command_;

		// dynamic reconfigure
		dynamic_reconfigure::Server<holodeck_controller::frontendConfig> server_;


		// Frames
		cv::Mat img_;
		cv::Mat grayImg_;
		cv::Mat alteredImg_;

		// PD controller
		struct PD_Controller {
			float kp;    // proportional gain
			float kd;    // derivative gain
			float c;     // command
			float prev;  // previous value
			float dl;    // derivative
		} Vx, Vy, Vyaw;

		float yaw_stop_; // yaw angle when yaw_rate was last zero 

		// If true, control using the keyboard
		bool use_keyboard_ = true;

		// Extenstions
		// ControllerManager controller_manager_;
		std::shared_ptr<ControllerBase> controller_base_;
		VisualOdometry visual_odometry_;
		FeatureManager feature_manager_;

		// Used to indicate how many images to throw away
		unsigned img_count_;
		unsigned img_use_; // Throw out img_use_ out of img_use_+1 images



		// Retrieve Holodeck Image
		void callback_sub_video(const sensor_msgs::ImageConstPtr& data);

		// publish commands to holodeck
		void publish_command();

		// get state information
		void service_state();

		// implement controller
		void implement_extensions();

		void handle_key_input(const keyboard_input::keyboard& msg);

		// PD velocity controller. 
		// Calculates desired roll, pitch, and yaw_rate
		void pd_controller();

		// Update PD gains
		void rqt_reconfigure_callback(holodeck_controller::frontendConfig &config, uint32_t level);

		void implement_visual_odometry();

	} ;







}
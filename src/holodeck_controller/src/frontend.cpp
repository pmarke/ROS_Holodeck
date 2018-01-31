#include "holodeck_controller/frontend.h"


namespace holodeck {

	Frontend::Frontend() {

		ros::NodeHandle nh_private("controller");

		// ROS communication
		image_transport::ImageTransport it(nh_);
		pub_video_ = it.advertise("altered_video",10);
		sub_video_ = it.subscribe("video", 10, &Frontend::callback_sub_video,this,image_transport::TransportHints("raw"));
		command_pub_ = nh_.advertise<ros_holodeck::command>("command",1);
		srv_state_ = nh_.serviceClient<ros_holodeck::state_srv>("state");

		controller_base_ = std::make_shared<ControllerOptFlow>();

		keyboard_input();

	}


	void Frontend::callback_sub_video(const sensor_msgs::ImageConstPtr& data) {
		
		// Get the image
		try{
			img_ = cv_bridge::toCvCopy(data,sensor_msgs::image_encodings::BGR8)->image;
		}
		catch(cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge execption: %s", e.what());
			return;
		}

		// Get the states
		service_state();

		implement_extensions();

		if (!use_keyboard_) {

			// Implement extensions
			implement_extensions();

			command_.yaw_rate = 0;
			command_.pitch = 0;
			command_.roll = 0;
			publish_command();
		}


	}

	void Frontend::publish_command() {

		command_pub_.publish(command_);

	}

	void Frontend::service_state() {

		if (srv_state_.call(state_srv_)) {

			state_ = state_srv_.response.state;
		}
		else
		{
			ROS_ERROR("Holodeck Frontend: Failed to call service state");
		}

	}

	void Frontend::implement_extensions() {


		cv::cvtColor(img_, grayImg_, cv::COLOR_BGR2GRAY);
		controller_base_->implement_controller(grayImg_, state_, command_);

	}

	void Frontend::keyboard_input() {


		initscr();
		cbreak(); //Disable buffering
		noecho(); // supress echo
		keypad(stdscr,TRUE); // adds special keystrokes like backspace, delete, and the arrow keys
		nodelay(stdscr,TRUE); // allows getch() to work in a non-blocking manner

		ros::Rate loop_rate(30);

		while(ros::ok())
		{



		  int key_pressed;
		  if((key_pressed=getch())==ERR){
		    // user hasn't responded. do nothing
		  }
		  else{
		    handle_key_input(key_pressed);
		  }


		  ros::spinOnce();
		  loop_rate.sleep();
		}

	}

	void Frontend::handle_key_input(int key_pressed) {

		command_.reset = false;


		switch(key_pressed) {

			case key_w: { // increase altitude

				command_.altitude += 1;

				break;
			}
			case key_a: {// yaw rate left

				command_.yaw_rate -= 0.1;

				if (command_.yaw_rate < -1)
					command_.yaw_rate = -1;


				break;
			}
			case key_s: { // decrease altitude

				command_.altitude -= 1;

				if (command_.altitude < 0)
					command_.altitude = 0;

				break;
			}
			case key_d: { // yaw_rate right

				command_.yaw_rate += 0.1;

				if (command_.yaw_rate > 1)
					command_.yaw_rate = 1;

				break;
			}
			case key_r: { // reset

				command_.reset = true;
				command_.altitude = 0;

				break;
			}
			case key_c: { // change between publishing commands with keyboard and controller

				use_keyboard_ = !use_keyboard_;

				break;
			}
			case key_arrow_up: { // pitch up

				command_.pitch += 0.1;

				if (command_.pitch > 0.7)
					command_.pitch = 0.7;

				break;
			}
			case key_arrow_down: { // pitch down

				command_.pitch -= 0.1;

				if (command_.pitch < -0.4)
					command_.pitch = -0.4;

				break;
			}
			case key_arrow_left: { // roll left

				command_.roll -= 0.1;

				if (command_.roll < -0.4)
					command_.roll = -0.4;

				break;
			}
			case key_arrow_right: { // roll right

				command_.roll += 0.1;

				if (command_.roll > 0.4)
					command_.roll = 0.4;

				break;
			}
			case key_space: {  // reset commands to 0

				command_.pitch = 0;
				command_.roll = 0;
				command_.yaw_rate = 0;

				break;
			}
			case key_q: { // step forward with current command

				break;
			}
			default: {

				ROS_WARN("Key: %i not recognized\n", key_pressed);

				break;
			}
		}

		if (use_keyboard_)
			publish_command();

	}






}
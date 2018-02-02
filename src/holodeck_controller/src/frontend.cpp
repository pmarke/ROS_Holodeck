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

		// rqt_reconfigure
		auto f = std::bind(&Frontend::rqt_reconfigure_callback, this, std::placeholders::_1, std::placeholders::_2);
		server_.setCallback(f);


		// initial values;
		Vx.c = 0;
		Vx.prev = 0;

		Vy.c = 0;
		Vy.prev = 0;

		Vyaw.c = 0;
		Vyaw.prev = 0;
		yaw_stop_ = 0;


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

		// implement controller
		pd_controller();

		// change coordinate frame
		ros_holodeck::command command;

		command.yaw_rate = -command_.yaw_rate;
		command.roll = 0;//-command_.roll;
		command.altitude = command_.altitude;
		command.pitch = command_.pitch;
		command.reset = command_.reset;

		command_pub_.publish(command);

	}

	void Frontend::service_state() {

		if (srv_state_.call(state_srv_)) {

			state_ = state_srv_.response.state;

			std::cout << "yaw:  " << state_.yaw << std::endl;
			std::cout << "command yaw_rate c: " << command_.yaw_rate << std::endl;
			std::cout << "Vyaw.c: " << Vyaw.c << std::endl;
			std::cout << "yaw_stop_: " << yaw_stop_ << std::endl;
			
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
		  while ( (key_pressed=getch()) != ERR ) {
		    // user hasn't responded. do nothing

		  	// std::cout << key_pressed << std::endl;

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

				command_.altitude += 1.0/30;

				break;
			}
			case key_a: {// yaw rate left

				Vyaw.c -= 0.1;

				if (Vyaw.c < -1)
					Vyaw.c = -1;


				break;
			}
			case key_s: { // decrease altitude

				command_.altitude -= 1.0/30;

				if (command_.altitude < 0)
					command_.altitude = 0;

				break;
			}
			case key_d: { // yaw_rate right

				Vyaw.c += 0.1;

				if (Vyaw.c > 1)
					Vyaw.c = 1;

				break;
			}
			case key_r: { // reset

				command_.reset = true;
				command_.altitude = 0;
				Vx.c = 0;
				Vy.c = 0;
				Vyaw.c = 0;

				break;
			}
			case key_c: { // change between publishing commands with keyboard and controller

				use_keyboard_ = !use_keyboard_;

				break;
			}
			case key_arrow_up: { // pitch up

				Vx.c += 0.1;

				if (Vx.c > 5)
					Vx.c = 5;

				break;
			}
			case key_arrow_down: { // pitch down

				Vx.c -= 0.1;

				if (Vx.c < -5)
					Vx.c = -5;

				break;
			}
			case key_arrow_left: { // roll left

				Vy.c -= 0.1;

				if (Vy.c < -5)
					Vy.c = -5;

				break;
			}
			case key_arrow_right: { // roll right

				Vy.c += 0.1;

				if (Vy.c > 5)
					Vy.c = 5;

				break;
			}
			case key_space: {  // reset commands to 0

				Vx.c = 0;
				Vy.c = 0;
				Vyaw.c = 0;

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

void Frontend::pd_controller() {


	// calculate derivatives
	Vx.dl = (state_.vel.x - Vx.prev) * frame_rate;
	Vy.dl = (state_.vel.y - Vy.prev) * frame_rate;

	// compute commands
	command_.pitch = Vx.kp*(Vx.c - state_.vel.x) - Vx.dl*Vx.kd;

	command_.roll = Vy.kp*(Vy.c-state_.vel.y) -Vy.dl*Vy.kd;

	if (Vyaw.c == 0 && Vyaw.dl != 0) {
		yaw_stop_ = state_.yaw;
	}

	if (Vyaw.c == 0) {

		command_.yaw_rate = Vyaw.kp*(yaw_stop_ - state_.yaw) - Vyaw.dl*Vyaw.kd;

	}
	else
		command_.yaw_rate = Vyaw.c;

	// store previous value
	Vx.prev = state_.vel.x;
	Vy.prev = state_.vel.y;
	Vyaw.dl = Vyaw.c;


}


void Frontend::rqt_reconfigure_callback(holodeck_controller::frontend &config, uint32_t level) {

	// update gains
	Vx.kp = config.pitch_kp;
	Vx.kd = config.pitch_kd;

	Vy.kp = config.roll_kp;
	Vy.kd = config.roll_kd;

	Vyaw.kp = config.yaw_kp;
	Vyaw.kd = config.yaw_kd;


}







}
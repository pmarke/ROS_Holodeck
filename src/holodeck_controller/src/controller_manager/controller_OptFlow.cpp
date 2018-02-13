#include "holodeck_controller/controller_manager/controller_OptFlow.h"

namespace holodeck {

ControllerOptFlow::ControllerOptFlow() { 



	left_wall_.x_start = 15;
	left_wall_.x_stop = image_width_/4.0;
	left_wall_.y_start = image_height_/4.0;
	left_wall_.y_stop = image_height_*(1.0-1.0/4.0);

	right_wall_.x_start = image_width_-15;
	right_wall_.x_stop = image_width_*(1 - 1.0/4.0);
	right_wall_.y_start = left_wall_.y_start;
	right_wall_.y_stop = left_wall_.y_stop;

	left_center_.x_start = image_width_/4.0;
	left_center_.x_stop = image_width_/2;
	left_center_.y_start = image_height_/4.0;
	left_center_.y_stop = image_height_/2.0;

	right_center_.x_start = image_width_/2;
	right_center_.x_stop = image_width_*(1.0-1.0/4.0);
	right_center_.y_start = left_center_.y_start;
	right_center_.y_stop = left_center_.y_stop;

	bottom_.x_start = image_width_/4;
	bottom_.x_stop = image_width_*(1.0-1.0/4.0);
	bottom_.y_start = image_height_*(1.0 - 1.0/3.0);
	bottom_.y_stop = image_height_-50;

	center_.x_start = 226;
	center_.x_stop = 286;
	center_.y_start = 206;
	center_.y_stop = 356;

	init_points(&left_wall_);
	init_points(&right_wall_);
	init_points(&right_center_);
	init_points(&left_center_);
	init_points(&bottom_);
	init_points(&center_);


 }


void ControllerOptFlow::implement_controller(const cv::Mat& img, const ros_holodeck::state state, float *command ) {

	// convert image to gray image
	cv::Mat grayImg;
	cv::cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);

	// Need at least two images to compute optical flow
	if (!first_image_) {

		compute_optical_flow(grayImg, state);

		calculate_commands(grayImg, state,command);

		// display the image
		display_image(img, command);

	}
	else
		first_image_ = false;

	// set new image to previous
	prev_img_ = grayImg.clone();

}

void ControllerOptFlow::compute_optical_flow(const cv::Mat& img, const ros_holodeck::state state) {

	find_correspoinding_points(img,&left_wall_);
	find_correspoinding_points(img,&right_wall_);
	find_correspoinding_points(img,&right_center_);
	find_correspoinding_points(img,&left_center_);
	find_correspoinding_points(img,&bottom_);
	find_correspoinding_points(img,&center_);

	compute_pixel_velocity(&left_wall_, state);
	compute_pixel_velocity(&right_wall_, state);
	compute_pixel_velocity(&right_center_, state);
	compute_pixel_velocity(&left_center_, state);
	compute_pixel_velocity(&bottom_, state);
	compute_pixel_velocity(&center_, state);
}


// // detect features on the new image using GFTT
// void ControllerOptFlow::detect_features(const cv::Mat& img) {

// 	// clear history
// 	prev_features_.clear();
// 	std::vector<cv::KeyPoint> keypoints;

// 	gftt_->detect(img, keypoints);

// 	// convert features to 2d points points
// 	for(int i = 0; i < keypoints.size(); i++)
// 		prev_features_.push_back(keypoints[i].pt);

// }

void ControllerOptFlow::find_correspoinding_points(const cv::Mat& img,RegionOfInterest* roi) {



	//clear history

	roi->matched_points.clear();
	roi->good_points.clear();

	std::vector<unsigned char> status;
	std::vector<float> err;
	std::vector<cv::Point2f> temp_next_features;

	// The pyramids used in opticalFlow
	std::vector<cv::Mat> pyramid1, pyramid2;

	cv::buildOpticalFlowPyramid(img, pyramid1, cv::Size(21,21),3);
	cv::buildOpticalFlowPyramid(prev_img_, pyramid2, cv::Size(21,21),3);

	cv::calcOpticalFlowPyrLK(pyramid2, 
							 pyramid1, 
							 roi->points, 
							 temp_next_features, 
							 status, 
							 err, 
							 cv::Size(21,21),
							 3,
							 cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,20,0.03),
							 0,
							 1e-4);

	// remove features that didn't have a match
	std::vector<cv::Point2f> temp_prev_features;


	for(int i = 0; i < status.size(); i++)
		if(status[i])
		{
			roi->good_points.push_back(roi->points[i]);
			roi->matched_points.push_back(temp_next_features[i]);
		}	


}

void ControllerOptFlow::compute_pixel_velocity(RegionOfInterest* roi, const ros_holodeck::state state) {



	roi->pixel_velocity.clear();

	for (int i = 0; i < roi->good_points.size(); i++) {

		// float d = sqrt(powf(roi->good_points[i].x-image_width_/2,2)+focal_length_2);
		// float theta = cos(focal_length_/d);
		// float s_temp = state.imu.gyro_z*d;
		// float s_dot = s_temp*acos(theta)*s_temp;

		// roi->pixel_velocity.push_back((roi->matched_points[i]-roi->good_points[i] )*frames_per_second_ + cv::Point2f(1,0)*state.imu.gyro_z*focal_length_*0.63);
		roi->pixel_velocity.push_back((roi->matched_points[i]-roi->good_points[i] )*frames_per_second_);

		// std::cout << " opt flow: " << (roi->matched_points[i]-roi->good_points[i] )*frames_per_second_ << "\r" << std::endl;
		// std::cout << "comp: " << cv::Point2f(1,0)*state.imu.gyro_z*focal_length_ << "\r" << std::endl;
	
	}

}

void ControllerOptFlow::compute_avg_optical_flow(RegionOfInterest* roi) {

	// Clear the history
	roi->count = 0;
	roi->sum = 0;
	roi->avg = 0;

	// Sum the optical flow in the x direction. Ignore outliers
	for (int i = 0; i < roi->pixel_velocity.size(); i++) {

		// Ignore outliers
		if (fabs(roi->pixel_velocity[i].x) < 500) {
			roi->count ++;
			roi->sum += fabs(roi->pixel_velocity[i].x);
		}
	}

	// Compute the average
	if (roi->count > 0) {
		roi->avg = roi->sum/roi->count;
	}
}

void ControllerOptFlow::calculate_height(const ros_holodeck::state state) {

	float altitude_sum = 0; 
	float new_avg = 0;
	

	for (int i = 0; i < bottom_.matched_points.size(); i++) {

		// distance the pixes is from the center of the image
		float s = (bottom_.good_points[i].y-image_height_/2);

		// Angle between center of image and y point. 
		float angle = atan(s/focal_length_);

		// velocity of the point on the image plane (pixels/s)
		float s_dl = (bottom_.matched_points[i].y - bottom_.good_points[i].y)*frames_per_second_;

		// Distance the point on the image plane is from the focal
		float d = sqrtf(pow(s,2) + focal_length_2);

		float D = state.vel.x*d*sin(angle)*cos(angle)/s_dl;

		float theta = angle - state.pitch;


		float altitude = D*sin(theta); //sin(angle)*d;

		altitude_sum += altitude;

	}

	new_avg = altitude_sum/bottom_.matched_points.size();

	// Throw out outliers. 
	if (fabs(new_avg -altitude_avg_ ) < 2) {

		// low pass filter
		altitude_avg_ = altitude_avg_*0.6 + 0.4*new_avg;

	}


	// std::cout << "h: " << altitude_avg_ << "\r" << std::endl;

}

void ControllerOptFlow::estimate_time_till_collision() {

	float tau = 0; // time till collision
	int count = 0;
	

	for (int i = 0; i <center_.good_points.size(); i++ ) {

		// distance the pixel is from the center of the image
		float s = center_.good_points[i].x - image_width_/2;

		if ( fabs(center_.pixel_velocity[i].x ) > 0.1) {

			tau += -s/center_.pixel_velocity[i].x;
			count++;
		}

	}

	if (tau < 0)
		time_till_collision_avg_ = time_till_collision_avg_*0.8 + tau*0.2/count;

	if (time_till_collision_avg_ < -10) {
		ttc_valid_ = true;
	}
	// std::cout << "tau: " << time_till_collision_avg_ << "\r" << std::endl;

}

void ControllerOptFlow::calculate_commands(const cv::Mat& img, const ros_holodeck::state state,float *command) {

	
	// Get the average optical flow over the ROIs
	compute_avg_optical_flow(&left_wall_);
	compute_avg_optical_flow(&right_wall_);
	compute_avg_optical_flow(&left_center_);
	compute_avg_optical_flow(&right_center_);
	compute_avg_optical_flow(&bottom_);

	// Calculate height
	calculate_height(state);

	// Calculate estimated time till collision
	if (state.vel.x > 2.5 && fabs(state.imu.gyro_x) < 0.1 && fabs(state.imu.gyro_y) < 0.2 && fabs(state.imu.gyro_z) < 0.1 && controller_state_ != AVOID_OBSTACLE && controller_state_ != STOP)
		estimate_time_till_collision();


	diff_wall_opt_flow_ = left_wall_.avg -right_wall_.avg;
	diff_center_opt_flow_ = left_center_.avg - right_center_.avg;

	if (state_timer_ > frames_per_second_) {
		can_change_states_ = true;
	}
	else 
	{
		state_timer_ ++;
		can_change_states_ = false;
	}
	



	// You are about to collide head on
	if (fabs(time_till_collision_avg_) < 2 && fabs(time_till_collision_avg_) > 0.1  && ttc_valid_&& (controller_state_ == STOP || can_change_states_)) {

		// Stop the copter
		command[0] = 0;
		command[1] = 0;
		command[2] = 0;

		// Change state
		if (controller_state_ != STOP) {
			controller_state_ = STOP;
			state_timer_	= 0; // reset timer
		}
	}
	// avoid obstacles by yawing
	else if (fabs(diff_center_opt_flow_) > 11 && (controller_state_ == AVOID_OBSTACLE || can_change_states_)) {

		command[2] = diff_center_opt_flow_/40.0;

		// saturate command
		if (fabs(command[2]) > 9) {
			command[2] = std::copysign(0.2, command[2]);
		}

		// Don't pitch or roll
		command[0] = 0;
		command[1] = 0;

		// Change state
		if (controller_state_ != AVOID_OBSTACLE) {
			controller_state_ = AVOID_OBSTACLE;
			state_timer_	= 0; // reset timer
		}


	}
	// Use roll command to guide copter down corridor
	else if (fabs(diff_wall_opt_flow_) > 4 && (controller_state_ == CORRIDOR_BALANCE || can_change_states_)) {

		command[1] = diff_wall_opt_flow_/4;

		// saturate command
		if (fabs(command[1]) > 0.5)
			command[1] = std::copysign(0.5,command[1]);

		// move forward but dont yaw
		command[0] = 4;
		command[2] = 0;

		// Change state
		if (controller_state_ != CORRIDOR_BALANCE) {
			controller_state_ = CORRIDOR_BALANCE;
			state_timer_	= 0; // reset timer
		}

	}
	// move forward
	else {
		command[0] = 4;
		command[1] = 0;
		command[2] = 0;

		// Change state
		if (controller_state_ != MOVE_FORWARD) {
			controller_state_ = MOVE_FORWARD;
			state_timer_	= 0; // reset timer
		}
	}

	// std::cout << "center diff: " << diff_center_opt_flow_ << "\r" << std::endl;
	// std::cout << "left center: " << left_center_.avg << "\r" << std::endl;
	// std::cout << "right center: " << right_center_.avg << "\r" << std::endl;
	std::cout << "wall diff: " << diff_wall_opt_flow_ << "\r" << std::endl;









	
	

}

void ControllerOptFlow::draw_optical_flow(cv::Mat& img, RegionOfInterest* roi) {

	for( int i = 0; i < roi->pixel_velocity.size(); i++) {

			cv::arrowedLine(img,
			 roi->good_points[i],
			 roi->good_points[i]+roi->pixel_velocity[i]/5, 
			 cv::Scalar(255,0,0), 3 );
	}

}

void ControllerOptFlow::display_image(const cv::Mat& img, float *command) {

	cv::Mat altered_img = img.clone();

	draw_optical_flow(altered_img, &left_wall_);
	draw_optical_flow(altered_img, &right_wall_);
	draw_optical_flow(altered_img, &left_center_);
	draw_optical_flow(altered_img, &right_center_);
	draw_optical_flow(altered_img, &bottom_);

	std::stringstream ss1,ss2;
	ss1 << std::setprecision(3) << altitude_avg_;
	ss2 << std::setprecision(3) << -time_till_collision_avg_;

	std::string state_text = "State: ";
	std::string altitude = "Altitude: " + ss1.str() + " (m)";
	std::string ttc = "Time Till Collision: " + ss2.str() + " (s)";


	switch (controller_state_) {

		case STOP:
		{
			state_text += "STOP";

			break;
		}
		case AVOID_OBSTACLE:
		{

			state_text += "AVOID_OBSTACLE";

			break;
		}
		case CORRIDOR_BALANCE:
		{

			state_text += "CORRIDOR_BALANCE";

			break;
		}
		case MOVE_FORWARD:
		{

			state_text += "MOVE_FORWARD";

			break;
		}
		case USER_CONTROL:
		{

			state_text += "USER_CONTROL";

			break;
		}
	}

	// Draw  text
	cv::rectangle(altered_img, cv::Point(image_width_-210, 5), cv::Point(image_width_ -10,70), cv::Scalar(255,255,255),CV_FILLED);
	cv::putText(altered_img, state_text.c_str(),cv::Point(image_width_-200, 20),CV_FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,0,0));
	cv::putText(altered_img, altitude.c_str(),cv::Point(image_width_-200, 40),CV_FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,0,0));
	cv::putText(altered_img, ttc.c_str(),cv::Point(image_width_-200, 60),CV_FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,0,0));


	// add velocity commands
	cv::arrowedLine(altered_img, cv::Point(50,50), cv::Point(50+diff_wall_opt_flow_, 50), cv::Scalar(0,255,255),3 );

	cv::imshow("optical flow", altered_img);
	cv::waitKey(1);

}

void ControllerOptFlow::init_points(RegionOfInterest* roi) {

	int delta_x = roi->x_stop - roi->x_start;
	int delta_y = roi->y_stop - roi->y_start;
	int increment_x = delta_x/6;
	int increment_y = delta_y/6;

	int x_start,x_stop;
	

	if (roi->x_start > roi->x_stop) {
		for (int c = roi->x_start; c > roi->x_stop; c += increment_x) {
			for (int r = roi->y_start; r < roi->y_stop; r+=increment_y) {
				roi->points.push_back(cv::Point2f(c,r));
			}
		}
	}
	else {
		for (int c = roi->x_start; c < roi->x_stop; c += increment_x) {
			for (int r = roi->y_start; r < roi->y_stop; r+=increment_y) {
				roi->points.push_back(cv::Point2f(c,r));
			}
		}

	}


	

}

}
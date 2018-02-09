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
	left_center_.y_start = left_wall_.y_start;
	left_center_.y_stop = left_wall_.y_stop;

	right_center_.x_start = image_width_/2;
	right_center_.x_stop = image_width_*(1.0-1.0/4.0);
	right_center_.y_start = left_wall_.y_start;
	right_center_.y_stop = left_wall_.y_stop;

	bottom_.x_start = image_width_/4;
	bottom_.x_stop = image_width_*(1.0-1.0/4.0);
	bottom_.y_start = left_wall_.y_stop;
	bottom_.y_stop = image_height_-50;

	init_points(&left_wall_);
	init_points(&right_wall_);
	init_points(&right_center_);
	init_points(&left_center_);
	init_points(&bottom_);


 }


void ControllerOptFlow::implement_controller(const cv::Mat& img, const ros_holodeck::state state, float *command ) {

	// Need at least two images to compute optical flow
	if (!first_image_) {

		compute_optical_flow(img);

		calculate_commands(img, state,command);

		// display the image
		display_image(img, command);

	}
	else
		first_image_ = false;

	// set new image to previous
	prev_img_ = img.clone();

}

void ControllerOptFlow::compute_optical_flow(const cv::Mat& img) {

	find_correspoinding_points(img,&left_wall_);
	find_correspoinding_points(img,&right_wall_);
	find_correspoinding_points(img,&right_center_);
	find_correspoinding_points(img,&left_center_);
	find_correspoinding_points(img,&bottom_);

	compute_pixel_velocity(&left_wall_);
	compute_pixel_velocity(&right_wall_);
	compute_pixel_velocity(&right_center_);
	compute_pixel_velocity(&left_center_);
	compute_pixel_velocity(&bottom_);
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

void ControllerOptFlow::compute_pixel_velocity(RegionOfInterest* roi) {

	roi->pixel_velocity.clear();

	for (int i = 0; i < roi->good_points.size(); i++) {

		roi->pixel_velocity.push_back((roi->matched_points[i]-roi->good_points[i])*frames_per_second_);
	
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
		if (roi->pixel_velocity[i].x < 15) {
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
		altitude_avg_ = altitude_avg_*0.4 + 0.6*new_avg;

	}


	// std::cout << "h: " << altitude_avg_ << std::endl;

}

void ControllerOptFlow::estimate_time_till_collision() {

	float tau = 0; // time till collision

	for (int i = 0; i <left_center_.pixel_velocity.size(); i++ ) {

		// distance the pixes is from the center of the image
		float s = left_center_.good_points[i].x - image_width_/2;

		if ( fabs(left_center_.pixel_velocity[i].x ) > 0.1)
			tau += -s/left_center_.pixel_velocity[i].x/30;

	}

	time_till_collision_avg_ = time_till_collision_avg_*0.4 + tau*0.6;
	std::cout << "tau: " << time_till_collision_avg_ << std::endl;

}

void ControllerOptFlow::calculate_commands(const cv::Mat& img, const ros_holodeck::state state,float *command) {

	
	// Get the average optical flow over the ROIs
	compute_avg_optical_flow(&left_wall_);
	compute_avg_optical_flow(&right_wall_);
	compute_avg_optical_flow(&left_center_);
	compute_avg_optical_flow(&right_center_);
	compute_avg_optical_flow(&bottom_);

	diff_wall_opt_flow_ = left_wall_.avg -right_wall_.avg;

	command[2] = 0; // Vy (m/s)
	command[0] = 2;

	if (fabs(diff_wall_opt_flow_) > 0.1) {

		command[1] = diff_wall_opt_flow_*2/30;

		if (fabs(command[1]) > 0.5)
			command[1] = std::copysign(0.5,command[1]);
	}
	else {
		command[1] = 0;
	}

	// Calculate height
	calculate_height(state);
	estimate_time_till_collision();







	// std::cout << "Vy: " << command[1] << std::endl;
	// std::cout << "avg center feature: " << avg_center_features << std::endl;
	// std::cout << "avg wall feature: " << fabs(avg_wall_features_) << std::endl;
	// std::cout << "Vy: " << command[1] << std::endl;
	// std::cout << "wall_feature_right: " << wall_feature_right/wall_feature_right_count << std::endl;
	// std::cout << "wall_feature_left: : " << wall_feature_left/wall_feature_left_count << std::endl;
	// std::cout << "left count: " << wall_feature_left_count << std::endl;
	// std::cout << "right count: " << wall_feature_right_count << std::endl;
	

}

void ControllerOptFlow::draw_optical_flow(cv::Mat& img, RegionOfInterest* roi) {

	for( int i = 0; i < roi->pixel_velocity.size(); i++) {

			cv::arrowedLine(img,
			 roi->good_points[i],
			 roi->matched_points[i]+roi->pixel_velocity[i]/5, 
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
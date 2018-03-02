#include "visual_odometry/visual_odometry.h"



namespace holodeck {


VisualOdometry::VisualOdometry() {

	// Set Parameters
	frame_rate_ = 30.0;
	state_init_ = false;
	display_ = true;

	rot_cam2world_ << 0, 0, -1, 
					-1, 0,  0, 
					 0, -1, 0; 

}

void VisualOdometry::implement_visual_odometry(const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features, const ros_holodeck::state state) {


	calculate_relative_pose(prev_features, new_features, state);
	update_state(state);

	// if (display_)
		// display();


} 

void VisualOdometry::update_state(const ros_holodeck::state state) {


	// First time updateing
	if (!state_init_) {

		// Update position and rotation
		position_(0) = -state.position.y;
		position_(1) = state.position.z;
		position_(2) = -state.position.x;
		attitude_ = R_delta_;

		state_init_ = true;

	}
	else {

		// Xn+1 = Rn*Xn + Tn
		position_ = R_delta_*position_ + T_delta_;

		//Rn+1 = Rn+1*Rn
		attitude_ = R_delta_*attitude_;


	}

	std::cout << "Position: " << rot_cam2world_*position_ << "\r" << std::endl;
	std::cout << "Attitude: " << attitude_ << "\r" << std::endl;

}

void VisualOdometry::calculate_relative_pose(const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features,  const ros_holodeck::state state) {

	// Normalized T
	cv::Mat T_norm, E, R_delta;
	Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> T_n;

	// Find The essential Matrix
	E = cv::findEssentialMat(prev_features, new_features, focal_length_, cv::Point(image_width_/2, image_height_/2), cv::RANSAC, 0.99, 0.1 );

	// Recover pose to get R and T_norm
	int check = cv::recoverPose(E, prev_features, new_features, R_delta, T_norm, focal_length_, cv::Point(image_width_/2, image_height_/2));

	// Calculate the magnitude of the velocity vector
	float v_mag = sqrt( pow(state.velocity.x, 2) + pow(state.velocity.y,2) + pow(state.velocity.z,2));



	// Convert to Eigen
	cv::cv2eigen(T_norm, T_n);
	cv::cv2eigen(R_delta, R_delta_);

	// Scale T_norm
	// T_ = v_mag * Ts * T_norm
	T_delta_ = v_mag * T_n / frame_rate_*4;

	// Rotate from camera to world
	// T_delta_ = rot_cam2world_*T_delta_;
	// R_delta_ = rot_cam2world_*R_delta_;

	std::cout << "Check" << check << "\r" <<std::endl;

	std::cout << "R: " << R_delta << "\r" << std::endl;
	std::cout << "R eigen: " << R_delta_ << "\r" << std::endl;
	// std::cout << "Tn: " << T_n << "\r" <<std::endl;
	// std::cout << "Tnorm: " << T_norm << "\r" <<std::endl;
	// std::cout << "T_delta: " << T_delta_ << "\r" <<std::endl;
// 


}

void VisualOdometry::display() {

}

void VisualOdometry::draw_map() {

	// Initialize it if empty
	if (map_.empty()) {

		map_ = cv::Mat::zeros(500, 500, CV_8U);

	}

}




}
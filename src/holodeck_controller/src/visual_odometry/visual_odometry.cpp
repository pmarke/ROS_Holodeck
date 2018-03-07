#include "visual_odometry/visual_odometry.h"



namespace holodeck {


VisualOdometry::VisualOdometry() {

	// Set Parameters
	frame_rate_ = 30.0;
	state_init_ = false;
	display_ = true;

}

void VisualOdometry::implement_visual_odometry(const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features, const ros_holodeck::state state) {


	calculate_relative_pose(prev_features, new_features, state);
	update_state(state);

	if (display_)
		display(state);


} 

void VisualOdometry::update_state(const ros_holodeck::state state) {


	// First time updateing
	if (!state_init_) {

		G_w2c_ << 0, 0, 1, state.position.x,
		          1, 0, 0, state.position.y,
		          0, -1, 0, state.position.z,
				  0, 0, 0,       1  ;

		state_init_ = true;

	}
	else {

		G_w2c_ = G_w2c_*G_delta_;

	}

	// std::cout << "G_w2c_: " << G_w2c_ << "\r" << std::endl;



}

void VisualOdometry::calculate_relative_pose(const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features,  const ros_holodeck::state state) {

	// Normalized T
	cv::Mat T_norm, E, R_delta, R1, R2, t;
	Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> T,R;

	// Find The essential Matrix
	E = cv::findEssentialMat(prev_features, new_features, focal_length_, cv::Point(image_width_/2, image_height_/2), cv::RANSAC, 0.99, 0.1 );

	// Decompose the essential matrix
	cv::decomposeEssentialMat(E,R1,R2,t);

	// Recover pose to get R and T_norm
	int check = cv::recoverPose(E, prev_features, new_features, R_delta, T_norm, focal_length_, cv::Point(image_width_/2, image_height_/2));

	// Calculate the magnitude of the velocity vector
	float v_mag = sqrt( pow(state.velocity.x, 2) + pow(state.velocity.y,2) + pow(state.velocity.z,2));

	// Ensure that the R is not bogus
	if (cv::trace(R1)[0] > 2.5) {
		R1.copyTo(R_delta);
	}
	else {
		R2.copyTo(R_delta);
	}

	// Convert to Eigen
	cv::cv2eigen(T_norm, T);
	cv::cv2eigen(R_delta, R);

	// Scale T_norm
	// T_ = v_mag * Ts * T_norm
	T= v_mag * T / frame_rate_*2.5;

	// Compose homogenous matrix 
	// This maps points in frame old into frame new
	G_delta_ << R(0,0), R(0,1), R(0,2), T(0),
				R(1,0), R(1,1), R(1,2), T(1),
				R(2,0), R(2,1), R(2,2), T(2),
				   0  ,    0  ,    0  ,  1  ;
	// G_delta_ << 1, 0, 0, T(0),
	// 			0, 1, 0, T(1),
	// 			0, 0, 1, T(2),
	// 		    0, 0, 0,  1  ;

	// std::cout << "R: " << R_delta << "\r" << std::endl;
	// std::cout << "Tnorm: " << T_norm << "\r" <<std::endl;
	// std::cout << "G_delta: " << G_delta_ << "\r" << std::endl;


	// Invert homogenous matrix
	// This tells us how the camera is moving with respect to old frame
    G_delta_ = G_delta_.inverse().eval();


	// std::cout << "R: " << R_delta << "\r" << std::endl;
	// std::cout << "R eigen: " << R_delta_ << "\r" << std::endl;
	// std::cout << "Tn: " << T_n << "\r" <<std::endl;
	// std::cout << "Tnorm: " << T_norm << "\r" <<std::endl;
	// std::cout << "T_delta: " << T_delta_ << "\r" <<std::endl;
// 


}

void VisualOdometry::display(const ros_holodeck::state state) {

	// Update the map
	draw_map(state);

	cv::imshow("Map", map_);
	cv::waitKey(1);

}

void VisualOdometry::draw_map(const ros_holodeck::state state) {

	
	cv::Point2f new_pos;

	// Initialize it if empty
	if (map_.empty()) {

		map_ = cv::Mat::zeros(500, 500, CV_8U);
		new_pos = cv::Point2f(G_w2c_(0,3) + map_.cols/2,G_w2c_(1,3)+map_.rows/2);
		
	}
	else {

		new_pos = cv::Point2f(G_w2c_(0,3) + map_.cols/2,G_w2c_(1,3)+map_.rows/2);
		cv::line(map_, pos_dl_, new_pos, cv::Scalar(255,255,255),2);

	}

	// Write estimated and true position on map
	std::stringstream estimated_pose;
	estimated_pose << std::setprecision(2) << "Estimated: (" << G_w2c_(0,3) <<"," << G_w2c_(1,3) << ","<< G_w2c_(2,3) <<")" ;

	std::stringstream true_pose;
	true_pose << std::setprecision(2) << "True:      (" << state.position.x <<"," << state.position.y << ","<< state.position.z <<")" ;
	cv::rectangle(map_, cv::Point(0,0), cv::Point(300,45), cv::Scalar(0,0,0), CV_FILLED);
	cv::putText(map_,estimated_pose.str(), cv::Point(0,15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255));
	cv::putText(map_,true_pose.str(), cv::Point(0,30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255));


	// Update last position
	pos_dl_ = cv::Point2f(new_pos);


}




}
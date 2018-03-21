#include "visual_odometry/visual_odometry.h"



namespace holodeck {


VisualOdometry::VisualOdometry() {

	// Set Parameters
	frame_rate_ = 30.0;
	state_init_ = false;
	prev_init_ = false;
	display_ = true;
	use_truth_ = true;

	// obtacle parameters
	min_height_ = 10;
	max_distance_ = 50;
	fov_ = M_PI/4;
	life_ = 60;
	max_value_ = 140;

	obstacles_ = cv::Mat::zeros(500, 500, CV_8U);
	decrease_ = cv::Mat::ones(500,500,CV_8U)*max_value_/life_;

}

void VisualOdometry::implement_visual_odometry(const cv::Mat& proj_matrix, const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features, const ros_holodeck::state state) {

	if (!use_truth_)
		calculate_relative_pose(prev_features, new_features, state);


	update_state(state);

	// Undistort the points
	std::vector<cv::Point2f> pts1, pts2;
	cv::undistortPoints(prev_features, pts1, proj_matrix, cv::noArray());
	cv::undistortPoints(new_features, pts2, proj_matrix, cv::noArray());
	// pts1 = prev_features;
	// pts2 = new_features;

	if (prev_init_) {
	// reconstruct_3d(pts1,pts2);

	reconstruct_3d_HZ(pts1,pts2);

	}
	else {
		prev_init_ = true;
	}

	handle_obstacle_points(state,X_vec_);

	if (display_)
		display(state);


} 

void VisualOdometry::update_state(const ros_holodeck::state state) {

	if (!use_truth_) {
		// First time updateing
		if (!state_init_) {

			G_w2c_ << 0, 0, 1, state.position.x,
			          1, 0, 0, state.position.y,
			          0, -1, 0, state.position.z,
					  0, 0, 0,       1  ;

			state_init_ = true;

		}
		else {

			G_w2c_prev_ = G_w2c_;
			G_w2c_ = G_w2c_prev_*G_delta_;

		}
	}
	else {

		// Truth
		G_w2c_prev_ = G_w2c_;

		Eigen::Matrix3f R_c2b;
		R_c2b << 0, 0, 1, 
			          1, 0, 0, 
			          0, -1, 0;

		Eigen::Matrix<float,4,4> truth;
		
		for(int i = 0; i < 9; i++) {

			truth(i%3,i/3) = state.R[i];
		}

		truth.block<3,3>(0,0) = truth.block<3,3>(0,0)*R_c2b;

		

		truth(0,3) = state.position.x;
		truth(1,3) = state.position.y;
		truth(2,3) = state.position.z;
		truth(3,3) = 1;

		truth(3,0) = 0;
		truth(3,1) = 0;
		truth(3,2) = 0;

		G_w2c_ = truth;

	}

	


	// std::cout << "Prev :" << G_w2c_prev_ << std::endl;
	


	// std::cout << "R true: " << std::endl << R << std::endl;

	// std::cout << "R estimated: " <<std::endl << G_w2c_.block<3,3>(0,0)*temp.inverse() << std::endl;

	// std::cout << "G_w2c_: " << std::endl << G_w2c_ << std::endl;
	// std::cout << "truth: " << std::endl << truth << std::endl;

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

	// std::cout << "T_norm: " << std::endl << T_norm << std::endl;
	// std::cout << "R_delta: " << std::endl << R << std::endl;

	// Scale T_norm
	// T_ = v_mag * Ts * T_norm
	T= v_mag * T / frame_rate_*2.5;

	// Compose homogenous matrix 
	// This maps points in frame old into frame new
	G_delta_ << R(0,0), R(0,1), R(0,2), T(0),
				R(1,0), R(1,1), R(1,2), T(1),
				R(2,0), R(2,1), R(2,2), T(2),
				   0  ,    0  ,    0  ,  1  ;


	// Invert homogenous matrix
	// This tells us how the camera is moving with respect to old frame
    G_delta_ = G_delta_.inverse().eval();
 
    // Convert cv::Mat E to Eigen
   	cv::cv2eigen(E,E_);

	// std::cout << "E: " << std::endl << E << std::endl;


}

void VisualOdometry::reconstruct_3d(const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features) {



	Eigen::MatrixXf cam0, cam1;
	cv::Mat cv_cam0, cv_cam1;

	cam0 = (G_w2c_prev_.inverse().eval()).block<3,4>(0,0);
	cam1 = (G_w2c_.inverse().eval()).block<3,4>(0,0);

	// convert eigen to opencv
	cv::eigen2cv(cam0, cv_cam0);
	cv::eigen2cv(cam1, cv_cam1);

	cv::triangulatePoints(cv_cam0, cv_cam1, prev_features, new_features, points_3d_);

	// std::cout << "Points 3d" << std::endl << points_3d_ << std::endl;
	// std::cout << "rows: " << points_3d_.rows << std::endl;
	// std::cout << "cols: " << points_3d_.cols << std::endl;
	// std::cout << "size: " << points_3d_.size() << std::endl;

	for (int i = 0; i < points_3d_.cols; i++) {

		for (int j = 0; j < points_3d_.rows; j++) {
			points_3d_.at<float>(j,i) = points_3d_.at<float>(j,i)/points_3d_.at<float>(points_3d_.rows-1,i);
		}
	}

	// std::cout << "Points 3d" << std::endl << points_3d_ << std::endl;


	// std::cout << "features: " << new_features << std::endl << std::endl;
	// std::cout << "features: " << temp << std::endl << std::endl;




}

void VisualOdometry::reconstruct_3d_HZ(const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features) {


	if (use_truth_)
		reconstruct_essential_mat(G_w2c_prev_, G_w2c_, E_);

	// Instatiate 
	Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic>  P1, P2;
	

	// Compute P1 and P2
	P1 = (G_w2c_prev_.inverse().eval()).block<3,4>(0,0);
	P2 = (G_w2c_.inverse().eval()).block<3,4>(0,0);

	// World coordinate point
	Eigen::Vector4f X;

	// Clear history
	X_vec_.clear();

	for (int i = 0; i < new_features.size(); i++) {

		// Convert cv::Point2f to Eigen
		Eigen::Vector2f x1, x2;
		x1 << prev_features[i].x, prev_features[i].y;
		x2 << new_features[i].x, new_features[i].y;

		// std::cout << "x1: " << std::endl << x1 << std::endl;
		// std::cout << "x2: " << std::endl << x2 << std::endl;
		minimize_reprojection_error(E_, x1,x2);
		// std::cout << "x1: " << std::endl << x1 << std::endl;
		// std::cout << "x2: " << std::endl << x2 << std::endl << std::endl;

		// Get world point
		reconstruct_3d_HZ_point(P1,P2,x1,x2,X);

		X_vec_.push_back(X);


	}



}


void VisualOdometry::reconstruct_3d_HZ_point(const Eigen::Matrix<float,3,4>& P1, const Eigen::Matrix<float,3,4>& P2, const Eigen::Vector2f& x1, const Eigen::Vector2f& x2, Eigen::Vector4f& X) {

	// Construct the A matrix s.t. AX = 0
	Eigen::Matrix<float,4,4> A;
	A.block<1,4>(0,0) = x2(1)*P2.block<1,4>(2,0)- P2.block<1,4>(1,0);
	A.block<1,4>(1,0) = -x2(0)*P2.block<1,4>(2,0)+ P2.block<1,4>(0,0);
	A.block<1,4>(2,0) = x1(1)*P1.block<1,4>(2,0)- P1.block<1,4>(1,0);
	A.block<1,4>(3,0) = -x1(0)*P1.block<1,4>(2,0)+ P1.block<1,4>(0,0);
	
	

	// std::cout << "A: " << std::endl << A << std::endl;

	// SVD
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A,Eigen::ComputeThinU | Eigen::ComputeThinV);

	// Right Singular value
	Eigen::MatrixXf V = svd.matrixV();

	// std::cout << "V: " << V << std::endl;

	X = V.block<4,1>(0,3).transpose();
	// std::cout << "X: " << std::endl << X << std::endl;

	X = X/X(3);

	// std::cout << "X: " << std::endl << X << std::endl;

}

void VisualOdometry::minimize_reprojection_error(const Eigen::Matrix3f& E, Eigen::Vector2f& x1, Eigen::Vector2f& x2) {

	// Declare objects
	Eigen::Matrix3f T1, T2; // Translations matrices that takes the points the origin
	Eigen::Matrix3f E_temp; // Temp essential matrix to which the transformations will be applied
	Eigen::Vector3f e1, e2; // Epipoles
	Eigen::Matrix3f R1, R2; // Rotate the epipoles to the x axis

	// Create the translation matrices
	T1.setIdentity();
	T2.setIdentity();
	T1(0,2) = -x1(0);
	T1(1,2) = -x1(1);	
	T2(0,2) = -x2(0);
	T2(1,2) = -x2(1);

	// Update Essential matrix
	E_temp	= E;
	E_temp = T2.inverse().transpose()*E*T1.inverse();

	// Get the right and left epipoles using SVD
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(E_temp,Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXf V = svd.matrixV();
	Eigen::MatrixXf U = svd.matrixU();
	e1 = V.block<3,1>(0,2).transpose();
	e2 = U.block<3,1>(0,2);

	// scale epipoles s.t. e1(0)^2 + e1(1)^2 = 1
	float scale; 
	scale = 1.0/(sqrt( ( pow( e1(0),2 )+pow( e1(1),2 ) ) ) );
	e1 = e1*scale;
	scale = 1.0/(sqrt( ( pow( e2(0),2 )+pow( e2(1),2 ) ) ) );
	e2 = e2*scale;

	// Form the rotation matrices
	R1 << e1(0), e1(1), 0 , -e1(1), e1(0), 0, 0, 0, 1;
	R2 << e2(0), e2(1), 0 , -e2(1), e2(0), 0, 0, 0, 1;

	// Update E_temp
	E_temp = R2*E_temp*R1.transpose();

	// Create elements of E2
	float f1, f2, a, b, c, d;
	f1 = e1(2);
	f2 = e2(2);
	a = E_temp(1,1);
	b = E_temp(1,2);
	c = E_temp(2,1);
	d = E_temp(2,2);

	// a = 5;
	// b = 7;
	// c = 2;
	// d = 3;
	// f1 = 8;
	// f2 = 9;


	// Find the roots to the 6 order polynomial by constructing coefficients
	cv::Mat roots;
	cv::Mat coeff(1,7,CV_64FC1);
	coeff.at<double>(0,6) = a*a*c*d*pow(f1,4) - a*b*c*c*pow(f1,4);
	coeff.at<double>(0,5) = a*a*d*d*pow(f1,4) - 2*a*a*c*c*f2*f2 - b*b*c*c*pow(f1,4) - pow(c,4)*pow(f2,4) - pow(a,4);
	coeff.at<double>(0,4) = -4*a*a*c*d*f2*f2 + 2*a*a*c*d*f1*f1 + a*b*d*d*pow(f1,4) - 4*a*b*c*c*f2*f2 - 2*a*b*c*c*f1*f1 - b*b*c*d*pow(f1,4) - 4*pow(c,3)*d*pow(f2,4) - 4*pow(a,3)*b;
	coeff.at<double>(0,3) = -8*a*b*c*d*f2*f2 - 6*c*c*d*d*pow(f2,4) - 2*b*b*c*c*f2*f2 - 2*a*a*d*d*f2*f2 - 2*b*b*c*c*f1*f1 + 2*a*a*d*d*f1*f1 - 6*a*a*b*b;
	coeff.at<double>(0,2) = - 4*b*b*c*d*f2*f2 - 2*b*b*c*d*f1*f1 - 4*a*b*d*d*f2*f2 + 2*a*b*d*d*f1*f1 + a*a*c*d - 4*c*pow(d,3)*pow(f2,4) - a*b*c*c - 4*a*pow(b,3);
	coeff.at<double>(0,1) = - 2*b*b*d*d*f2*f2 + a*a*d*d - pow(d,4)*pow(f2,4) - b*b*c*c - pow(b,4);
	coeff.at<double>(0,0) = a*b*d*d - b*b*c*d;

	cv::solvePoly(coeff, roots, 300);

	// squared distance 
	auto s = [](const double& t, const double& a, const double& b, const double& c, const double& d, const double& f1, const double& f2){
		return pow(t,2)/(1+pow(f1,2)*pow(2,t)) + pow(c*t+d,2)/( pow(a*t+b,2) + pow(f2,2)*pow(c*t+d,2));
	};

	// Find the minimum coeff
	double min_dist; // minimum distance
	int index;       // index of minimum distance
	double t;        // the root that minimizes the function s
	for (int i = 0; i < 6; i++) {
		double distance = s(roots.at<double>(i,0),a,b,c,d,f1,f2);
		// std::cout << "distance: " << distance << std::endl;

		if (i == 0 || distance < min_dist) {
			min_dist = distance;
			index = i;
			t = roots.at<double>(i,0);
		}

	}

	// Form line equations
	Eigen::Vector3f l1, l2;
	l1 << t*f1, 1, -t;
	l2 << -f2*(c*t+d), a*t+b, c*t+d;

	// Get optimized points and transfer back into original coordinates
	Eigen::Vector3f x1_hat, x2_hat;
	x1_hat << -l1(0)*l1(2), -l1(1)*l1(2), pow(l1(0),2) + pow(l1(1),2);
	x2_hat << -l2(0)*l2(2), -l2(1)*l2(2), pow(l2(0),2) + pow(l2(1),2);

	x1_hat = T1.inverse()*R1.transpose()*x1_hat;
	x2_hat = T2.inverse()*R2.transpose()*x2_hat;
	x1_hat = x1_hat/x1_hat(2);
	x2_hat = x2_hat/x2_hat(2);

	// Pack into x1 and x2
	x1 = x1_hat.block<2,1>(0,0);
	x2 = x2_hat.block<2,1>(0,0);



	// std::cout << "min dist: " << min_dist << std::endl;
	// std::cout << "index: " << index << std::endl;


	// std::cout << "f1: "  << f1 << std::endl;
	// std::cout << "f2: "  << f2 << std::endl;
	// std::cout << "a: "  << a << std::endl;
	// std::cout << "b: "  << b << std::endl;
	// std::cout << "c: "  << c << std::endl;
	// std::cout << "d: "  << d << std::endl;



	// std::cout << "c0: "  << c0 << std::endl;
	// std::cout << "c1: "  << c1 << std::endl;
	// std::cout << "c2: "  << c2 << std::endl;
	// std::cout << "c3: "  << c3 << std::endl;
	// std::cout << "c4: "  << c4 << std::endl;
	// std::cout << "c5: "  << c5 << std::endl;
	// std::cout << "c6: "  << c6 << std::endl;

	// std::cout << "roots: " << roots << std::endl; 
	// std::cout << "roots: " << roots.at<double>(0,0) << std::endl; 




	


	// std::cout << "E_temp: " << std::endl << E_temp << std::endl;
	// std::cout << "V: " << std::endl << V << std::endl;
	// std::cout << "U: " << std::endl << U << std::endl;
	// std::cout << "e1: " << std::endl << e1 << std::endl;
	// std::cout << "e2: " << std::endl << e2 << std::endl;
	// std::cout << "R1: " << std::endl << R1 << std::endl;
	// std::cout << "R2: " << std::endl << R2 << std::endl;
	// std::cout << "test: " << std::endl << test << std::endl;



	// std::cout << "x1: " << std::endl << x1 << std::endl;
	// std::cout << "x2: " << std::endl << x2 << std::endl;
	// std::cout << "x1_hat: " << std::endl << x1_hat << std::endl;
	// std::cout << "x2_hat: " << std::endl << x2_hat << std::endl;
	// std::cout << "T1: " << std::endl << T1 << std::endl;
	// std::cout << "T2: " << std::endl << T2 << std::endl;

	// std::cout << "E_temp*e1: " << std::endl << E_temp*e1 << std::endl;
	// std::cout << "e2^T*E: " << std::endl << e2.transpose()*E_temp << std::endl;




}


void VisualOdometry::reconstruct_essential_mat(const Eigen::Matrix<float,4,4>& G_w2c_prev, const Eigen::Matrix<float,4,4>& G_w2c, Eigen::Matrix3f& E) {

	// reconstruct essential matrix
	// E = T_hat*R

	// Declare objects
	Eigen::MatrixXf G_delta, R, T;
	Eigen::Matrix3f T_hat;

	// Get the transformation between the two transformations
	G_delta = G_w2c_prev.inverse()*G_w2c;

	// Invert it to make it from frame 1 to frame 2
	G_delta = G_delta.inverse().eval();

	// Reconstruct R
	R = G_delta.block<3,3>(0,0);

	// Reconstruct T
	T = G_delta.block<3,1>(0,3);

	// Create skey symmetric matrix
	T_hat << 0, -T(2), T(1), T(2),0,-T(0),-T(1),T(0),0;

	// Reconstruct E
	E = T_hat * R;

}

void VisualOdometry::handle_obstacle_points(const ros_holodeck::state state, const std::vector<Eigen::Vector4f>& new_obstacles) {

	// For an obtacle to be added as reliable it must
	// the height must be above a threshold
	// It must be within a distance of the copter
	// It must be within the fov of the copter
	std::vector<Eigen::Vector4f> valid_obstacles;

	for (int i = 0; i < new_obstacles.size(); i++) {

		// See if obstacle is above min height
		if ( new_obstacles[i](2) > min_height_ ) {

			// Calculate distance between obstacle and copter
			float X = new_obstacles[i](0)-state.position.x;
			float Y = new_obstacles[i](1)- state.position.y;
			float d = sqrt( pow(X ,2) + pow(Y,2));

			// See if it is within the max distance
			if (d < max_distance_) {

				// See if the obstacle is withing the camera's fov
				float theta = atan2(Y,X);

				while (theta - state.yaw < -M_PI) {
					theta = theta + 2*M_PI;
				}
				while (theta -state.yaw > M_PI) {
					theta = theta - 2*M_PI;
				}

				if ( fabs(theta - state.yaw) < fov_) {

					valid_obstacles.push_back(new_obstacles[i]);
		
				}

			}

		}
	}

	update_obstacle_points(valid_obstacles);

}

void VisualOdometry::update_obstacle_points(const std::vector<Eigen::Vector4f>& new_obstacles) {

	int dim = 1;

	for (int i = 0; i < new_obstacles.size(); i++) {

		cv::Point point1(new_obstacles[i](0) + obstacles_.cols/2+dim, new_obstacles[i](1)+obstacles_.rows/2-dim);
		cv::Point point2(new_obstacles[i](0) + obstacles_.cols/2-dim, new_obstacles[i](1)+obstacles_.rows/2+dim);

		cv::rectangle(obstacles_, point1, point2, max_value_, CV_FILLED);

	}

	// decay life
	cv::subtract(obstacles_, decrease_, obstacles_); 
	
}



void VisualOdometry::display(const ros_holodeck::state state) {

	// Update the map
	draw_map(state);

	cv::Mat test = map_.clone();

	cv::add(test, obstacles_, test);


	cv::imshow("Map", test);
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
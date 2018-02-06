#include "holodeck_controller/controller_manager/controller_OptFlow.h"

namespace holodeck {

ControllerOptFlow::ControllerOptFlow() { 

	gftt_ = cv::GFTTDetector::create(100,0.05,25.0,6,false,0.05);

 }


void ControllerOptFlow::implement_controller(const cv::Mat& img, const ros_holodeck::state state, float *command ) {

	// Need at least two images to compute optical flow
	if (!first_image_) {

		compute_optical_flow(img);

		compute_pixel_velocity();

		calculate_commands(img, command);

		// display the image
		display_image(img, command);

	}
	else
		first_image_ = false;

	// get features from the newest image
	detect_features(img);

	// set new image to previous
	prev_img_ = img.clone();

}

// detect features on the new image using GFTT
void ControllerOptFlow::detect_features(const cv::Mat& img) {

	// clear history
	prev_features_.clear();
	std::vector<cv::KeyPoint> keypoints;

	gftt_->detect(img, keypoints);

	// convert features to 2d points points
	for(int i = 0; i < keypoints.size(); i++)
		prev_features_.push_back(keypoints[i].pt);

}

void ControllerOptFlow::compute_optical_flow(const cv::Mat& img) {


	//clear history

	next_features_.clear();

	std::vector<unsigned char> status;
	std::vector<float> err;
	std::vector<cv::Point2f> temp_next_features;

	// The pyramids used in opticalFlow
	std::vector<cv::Mat> pyramid1, pyramid2;

	cv::buildOpticalFlowPyramid(img, pyramid1, cv::Size(21,21),3);
	cv::buildOpticalFlowPyramid(prev_img_, pyramid2, cv::Size(21,21),3);

	cv::calcOpticalFlowPyrLK(pyramid2, 
							 pyramid1, 
							 prev_features_, 
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
			temp_prev_features.push_back(prev_features_[i]);
			next_features_.push_back(temp_next_features[i]);
		}

	prev_features_ = temp_prev_features;
	

	


}

void ControllerOptFlow::compute_pixel_velocity() {

	pixel_velocity_.clear();

	for (int i = 0; i < prev_features_.size(); i++) {

		pixel_velocity_.push_back(next_features_[i]-prev_features_[i]);
	
	}

}

void ControllerOptFlow::calculate_commands(const cv::Mat& img, float *command) {

	float count_wall_features = 0;    // Number of features detected on the wall ROI
	float count_center_features = 0;  // Number of features detected in the center ROI
	float sum_wall_features = 0;      // Sum of abs(opt_flow_wall_left) - abs(opt_flow_wall_right)
	float sum_center_features = 0;    // Sum of abs(opt_flow_center_left) - abs(opt_flow_center_right)
	float avg_wall_features = 0;      // = sum_wall_features / count_wall_features
	float avg_center_features = 0;    // = sum_center_features / count_center_features

	//                IMAGE
	///////////////////////////////////////////
	//                                       //
	//                                       //
	//                                       //
	///////////////////////////////////////////
	//         //  //         //  //         //
	//  ROI    //  //   ROI   //  //   ROI   //
	// LEFT    //  //  CENTER //  //  RIGHT  //
	// WALL    //  //         //  //  WALL   //
	//         //  //         //  //         //
	//         //  //         //  //         //
	///////////////////////////////////////////
	//           //   ROI       //           //
	//           //  BOTTOM     //           //
	//           //             //           //
	///////////////////////////////////////////




	// Grab the features in the different ROI and sum them. 
	for (int i = 0; i < prev_features_.size(); i++) {

		// Middle horizontal segment of image
		if ( (prev_features_[i].y > img.rows/5) && (prev_features_[i].y < img.rows*(1.0 - 1.0/5.0))) {

			// Left wall portion
			if (prev_features_[i].x < img.cols/4) {

				count_wall_features ++;
				sum_wall_features += abs(pixel_velocity_[i].x);

			}
			// Right wall portion
			else if (prev_features_[i].x > img.cols*(1.0-1.0/4.0)) {

				count_wall_features++;
				sum_wall_features -= abs(pixel_velocity_[i].x);


			}
			//  Center portion for collision avoidance
			else if (prev_features_[i].x > img.cols/4 && prev_features_[i].x < img.cols*(1.0-1.0/4.0)) {  

				// Left Center portion
				if (prev_features_[i].x > img.cols/2) {

					count_center_features ++;
					sum_center_features -= abs(pixel_velocity_[i].x);

				}
				// Right center portion
				else {

					count_center_features ++;
					sum_center_features += abs(pixel_velocity_[i].x);

				}


			}

		}




	}

	// Average the optical flow sums. 
	if (count_center_features != 0) {

		avg_center_features = sum_center_features / count_center_features;

	}

	if (count_wall_features != 0) {

		avg_wall_features = sum_center_features / count_wall_features;

	}

	command[0] = 5;                     // Vx = 3 (m/s)
	command[1] = 0; // Vy (m/s)

	if (abs(avg_wall_features) > abs(avg_center_features)*0.8) {

		command[2] = (avg_wall_features - avg_center_features)*10;  // Yaw_rate (m/s)

	}
	else {

		command[2] =  avg_center_features*10;  // Yaw_rate (m/s)

	}


	std::cout << "Vy: " << command[1] << std::endl;
	std::cout << "avg center feature: " << avg_center_features << std::endl;
	std::cout << "avg wall feature: " << avg_wall_features << std::endl;

	std::cout << "count center feture: " << count_center_features << std::endl;
	std::cout << "count wall feature: " << count_wall_features << std::endl;

}

void ControllerOptFlow::display_image(const cv::Mat& img, float *command) {

	cv::Mat altered_img = img.clone();

	// add arrows to the detected features
	for( int i = 0; i < next_features_.size(); i++) {

			cv::arrowedLine(altered_img,
			 prev_features_[i],
			 next_features_[i]+pixel_velocity_[i]*3, 
			 cv::Scalar(255,0,0), 3 );
	}

	// add velocity commands

	cv::arrowedLine(altered_img, cv::Point(50,50), cv::Point(50+command[2]*20, 50), cv::Scalar(0,255,255),3 );

	cv::imshow("optical flow", altered_img);
	cv::waitKey(1);

}

}
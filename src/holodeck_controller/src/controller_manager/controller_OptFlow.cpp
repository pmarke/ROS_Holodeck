#include "holodeck_controller/controller_manager/controller_OptFlow.h"

namespace holodeck {

ControllerOptFlow::ControllerOptFlow() { 

	gftt_ = cv::GFTTDetector::create(100,0.05,50.0,6,false,0.05);

 }


void ControllerOptFlow::implement_controller(const cv::Mat& img, const ros_holodeck::state state, ros_holodeck::command& command ) {

	// Need at least two images to compute optical flow
	if (!first_image_) {

		compute_optical_flow(img);

		compute_pixel_velocity();

	}
	else
		first_image_ = false;

	// display an image
	display_image(img);

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

void ControllerOptFlow::display_image(const cv::Mat& img) {

	cv::Mat altered_img = img.clone();

	// add arrows to the detected features
	for( int i = 0; i < next_features_.size(); i++) {

			cv::arrowedLine(altered_img,
			 prev_features_[i],
			 next_features_[i]+pixel_velocity_[i]*3, 
			 cv::Scalar(255,0,0), 3 );
	}

	cv::imshow("optical flow", altered_img);
	cv::waitKey(1);

}

}
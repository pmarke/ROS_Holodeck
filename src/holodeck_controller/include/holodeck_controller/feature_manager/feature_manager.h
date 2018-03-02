#pragma once

#include <opencv2/opencv.hpp>
#include <feature_manager/base_feature_matcher.h>
#include <feature_manager/lk_matcher.h>
#include <memory>


namespace holodeck{

class FeatureManager
{
public:
	FeatureManager();

	std::vector<cv::Point2f> prev_features_;    // Features obtained by a feature detection method on old frame
	std::vector<cv::Point2f> matched_features_; // Corresponding features on new frame
	std::vector<cv::Point2f> pixel_velocity_;   // pixel velocity between prev frame and new frame

	std::vector<cv::Point2f> moving_features_;    // Features that are moving
	std::vector<cv::Point2f> moving_velocity_;    // The velocity of the features that are moving


	// This method finds the corresponding features in the new frame
	// That were found the in previous frame.
	void find_correspoinding_features(const cv::Mat& img);

	cv::Mat prev_image_;

	cv::Mat mask_;
	cv::Mat homography_;



private:

	// Statistics with velocities
	float mean_;
	float stdd_;
	float k_;  

	float frame_rate_ = 1;

	bool display_;

	// Feature detector/tracker
	std::shared_ptr<BaseFeatureMatcher> base_feature_matcher_;

	// RANSAC inlier threshold
	const double ransac_thresh = 2.5f; 

	// calculates pixel velocity between frames of the detected features.
	void calculate_pixel_velocity();

	// Finds the pixel velocities above a certain standard deviation. 
	void get_moving_pixel_velocity();

	// Get the homography between the images
	void calculate_homography();

	// Dispay the moving features
	void display(const cv::Mat& img);


};



}


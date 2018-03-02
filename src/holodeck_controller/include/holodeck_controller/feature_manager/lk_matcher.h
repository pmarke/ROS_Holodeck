#pragma once

#include <opencv2/opencv.hpp>
#include "base_feature_matcher.h" 
#include <ros/ros.h>
#include <ros/console.h>

namespace holodeck {

	enum LK_Feature_Detector_Type:int {GFTT, FAST};

	class LK_Matcher : public BaseFeatureMatcher 
	{
	public:
		LK_Matcher();

		void find_correspoinding_features(const cv::Mat& img, const cv::Mat& prev_image, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& matched_features, const cv::Mat& mask);

	private:
		std::vector<cv::Point2f> prev_features_;
		std::vector<cv::Point2f> matched_features_;;


		LK_Feature_Detector_Type feature_detector_type_;

		cv::Mat prev_image_;

		

		bool display_;


		// gftt algorithm
		cv::Ptr<cv::GFTTDetector> gftt_;
		struct GFTT_Parameters{
			int maxCorners;          // Max number of features that will be detected
			double qualityLevel;     // Minimal accepted quality of image corners
			double minDistance;      // Minimum possible Euclidean distance between the returned corners
			int blockSize;        // Neighborhood size: block_size X block_size (pixels)
			bool useHarrisDetector;  // Harris detector free parameter
			double k;
		}gftt_parameters_;

		// fast parameters
		struct FAST_Parameters{
			int threshold;           // Threshold difference between intensity of the central pixel and the pixels of a circle around this pixel [0,255]
			bool nonmaxSuppression;  // If true, non-maximum suppression is applied to keypoints (features)
			int type;                // One of the three neighbors: FastFeatureDetector::TYPE_9_16, FastFeatureDetector::TYPE_7_12, FastFeatureDetector::TYPE_5_8
		} fast_parameters_;
		
		// LK_OpticalFlow_Parameters
		struct LK_OpticalFlow_Parameters{
			std::vector<unsigned char> status;      // each element of the vector is set to 1 if the flow for the corresponding feature has been found, otherwise 0
			std::vector<float> err;        // each element of the vector is set to an error for the corresponding feature
			cv::Size pyramid_size;         // size of the search window at each pyramid level
			int maxLevel;                  // 0-based maximal pyramid level; if set to zero, pyramids are not used, if set to 1, two levels, etc
			cv::TermCriteria termCriteria; // parameter specifying the termination criteria of the iterative search algorithm
			int flags;                     // cv::OPTFLOW_USE_INITIAL_FLOW, cv::OPTFLOW_LK_GET_MIN_EIGENVALS, or 0 
			double minEigThreshold;        // allows bad points to be removed

		} lk_opticalFlow_parameters_;


		// detect features by calling either detect_features_GFTT() or detect_features_FAST()
		// depending on the value of feature_detector_type_
		void detect_features(const cv::Mat& img, std::vector<cv::Point2f>& features);

		// find features using GFTT algorithm
		void detect_features_GFTT(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints);

		// find features using FAST algorithm
		void detect_features_FAST(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints);
	
		// implement optical flow
		void optical_flow(const cv::Mat& img, const cv::Mat& prev_image, std::vector<cv::Point2f>& next_features);
	
		// initial the gftt with its parameters
		void gfttInit();

		// display the found features
		void display(const cv::Mat& new_image, std::vector<cv::Point2f>& new_features, const cv::Mat& prev_image, std::vector<cv::Point2f>& prev_features);

		void draw_matches(const cv::Mat& new_image, std::vector<cv::Point2f>& new_features,const cv::Mat& prev_image, std::vector<cv::Point2f>& prev_features, cv::Mat& out_image);
	
		void draw_features(const cv::Mat& image, cv::Mat& out_image, std::vector<cv::Point2f>& features);

	};
}
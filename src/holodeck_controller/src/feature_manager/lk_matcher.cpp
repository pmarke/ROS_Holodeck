#include "feature_manager/lk_matcher.h"

namespace holodeck {

	LK_Matcher::LK_Matcher(){

		ros::NodeHandle nh("~");

		int feature_detector_type;

		// get parameters
		nh.param<int>("gftt_maxCorners", gftt_parameters_.maxCorners, 200);
		nh.param<double>("gftt_qualityLevel", gftt_parameters_.qualityLevel, 0.05);
		nh.param<double>("gftt_minDistance", gftt_parameters_.minDistance, 25.0);
		nh.param<int>("gftt_blockSize", gftt_parameters_.blockSize, 6);
		nh.param<bool>("gftt_useHarrisDetector", gftt_parameters_.useHarrisDetector, false);
		nh.param<double>("gftt_k", gftt_parameters_.k, 0.05);

		nh.param<int>("fast_threshold", fast_parameters_.threshold,100 );
		nh.param<bool>("fast_nonmaxSuppression", fast_parameters_.nonmaxSuppression, true );
		nh.param<int>("fast_type", fast_parameters_.type, cv::FastFeatureDetector::TYPE_9_16);
		
		nh.param<int>("LK_Feature_Detector_Type", feature_detector_type, (int)GFTT);
		feature_detector_type_ = static_cast<enum LK_Feature_Detector_Type>(feature_detector_type);

		nh.param("LK_OptFlow_maxLevel", lk_opticalFlow_parameters_.maxLevel,3);
		nh.param("LK_OptFlow_flags",lk_opticalFlow_parameters_.flags,0);
		nh.param("LK_OptFlow_minEigThreshold", lk_opticalFlow_parameters_.minEigThreshold,1e-4);
		lk_opticalFlow_parameters_.pyramid_size = cv::Size(21,21);
		lk_opticalFlow_parameters_.termCriteria = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,20,0.03);

		// Used for display
		display_ = true;
		// cv::namedWindow("LK_Matches", CV_WINDOW_NORMAL);
		// cv::resizeWindow("LK_Matches", 1680/3, 1050/3);
		// cv::moveWindow("Optical Flow", 0, 0);


		// Create gftt object with given parameters
		gfttInit();
	}

	void LK_Matcher::find_correspoinding_features(const cv::Mat& img, const cv::Mat& prev_image, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& matched_features, const cv::Mat& mask){

		// compute optical flow
		std::vector<cv::Point2f> matched_features_temp;

		cv::Mat tempImg;

		tempImg = img.clone();
		// cv::GaussianBlur(img, tempImg, cv::Size(5,5), 1.5);

		optical_flow(tempImg, prev_image, matched_features_temp);

		// only keep the good matches
		for(int i = 0; i < lk_opticalFlow_parameters_.status.size(); i++)
			if(lk_opticalFlow_parameters_.status[i])
			{
				prev_features.push_back(prev_features_[i]);
				matched_features.push_back(matched_features_temp[i]);
			}

		if (display_ && matched_features.size() > 0) 
			display(img,matched_features, prev_image, prev_features);

			
		// clear history
		prev_features_.clear();


			// get features of the new image
		detect_features(tempImg, prev_features_);

	

		

	}

	void LK_Matcher::detect_features(const cv::Mat& img, std::vector<cv::Point2f>& features){

		// temporary vector to hold keypoints
		std::vector<cv::KeyPoint> keypoints;

		// Depending on the feature detector type, grab features from image
		switch(feature_detector_type_){
			case GFTT:
			{
				detect_features_GFTT(img, keypoints);
				break;
			}
			case FAST:
			{
				detect_features_FAST(img, keypoints);
				break;
			}
			default :
			{
				ROS_ERROR("LK_MATCHER: Feature Detector Type not found. \n");
				break;
			}
		}

		// convert features to 2d points points
		for(int i = 0; i < keypoints.size(); i++)
			features.push_back(keypoints[i].pt);

	}

	void LK_Matcher::detect_features_GFTT(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints){

		//clear history
		keypoints.clear();

		// get the features
		gftt_->detect(img,keypoints);
	}


	void LK_Matcher::detect_features_FAST(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints){

		//clear history
		keypoints.clear();

		// get the features
		cv::FAST(img,keypoints,fast_parameters_.threshold, fast_parameters_.nonmaxSuppression, fast_parameters_.type);
	}

	void LK_Matcher::optical_flow(const cv::Mat& img, const cv::Mat& prev_image, std::vector<cv::Point2f>& next_features){


		// There has been at least two images
		if(first_image_ != true)
		{
			// clear history
			lk_opticalFlow_parameters_.status.clear();
			lk_opticalFlow_parameters_.err.clear();

			// The pyramids used in opticalFlow
			std::vector<cv::Mat> current_pyramids;
			cv::buildOpticalFlowPyramid(img, current_pyramids, lk_opticalFlow_parameters_.pyramid_size,lk_opticalFlow_parameters_.maxLevel);
			cv::calcOpticalFlowPyrLK(prev_image, 
									 img, 
									 prev_features_, 
									 next_features, 
									 lk_opticalFlow_parameters_.status, 
									 lk_opticalFlow_parameters_.err, 
									 lk_opticalFlow_parameters_.pyramid_size,
									 lk_opticalFlow_parameters_.maxLevel,
									 lk_opticalFlow_parameters_.termCriteria,
									 lk_opticalFlow_parameters_.flags,
									 lk_opticalFlow_parameters_.minEigThreshold);
	

		}
		else{
			first_image_ = false;
		}
	}

	void LK_Matcher::gfttInit(){

		gftt_ = cv::GFTTDetector::create();

		gftt_->setBlockSize(gftt_parameters_.blockSize);
		gftt_->setHarrisDetector(gftt_parameters_.useHarrisDetector);
		gftt_->setK(gftt_parameters_.k);
		gftt_->setMaxFeatures(gftt_parameters_.maxCorners);
		gftt_->setMinDistance(gftt_parameters_.minDistance);
		gftt_->setQualityLevel(gftt_parameters_.qualityLevel);
	}

	void LK_Matcher::display(const cv::Mat& new_image, std::vector<cv::Point2f>& new_features, const cv::Mat& prev_image, std::vector<cv::Point2f>& prev_features) {

		cv::Mat altered_image; //, altered_image2, altered_image3;

		// draw_features(prev_image, altered_image2, prev_features);
		draw_features(new_image, altered_image, new_features);

		// draw_matches(new_image,new_features, prev_image, prev_features, altered_image);

		// cv::imshow("prev features", altered_image2);
		cv::imshow("new features", altered_image);
		// cv::imshow("lk matches", altered_image);

	}

	void LK_Matcher::draw_matches(const cv::Mat& new_image, std::vector<cv::Point2f>& new_features,const cv::Mat& prev_image, std::vector<cv::Point2f>& prev_features, cv::Mat& out_image) {

		// concatenate the images
		cv::hconcat(prev_image, new_image, out_image);

		// Get the size of the prev_image
		cv::Size s = prev_image.size();

		// Get random color generator
		cv::RNG rng(12345);

		// Draw the lines
		for (int i = 0; i < prev_features.size(); i++) {


			cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));

			// Point offset
			cv::Point2f offset = cv::Point2f(s.width,0);

			// Draw line
			cv::line(out_image, prev_features[i], offset+new_features[i], color);

		}


	}

	void LK_Matcher::draw_features(const cv::Mat& image, cv::Mat& out_image, std::vector<cv::Point2f>& features) {

		// Clone the image
		out_image = image.clone();

		// add circles to the detected features
		for( int i = 0; i < features.size(); i++) {

				cv::circle(out_image,
				 features[i],5, 
				 cv::Scalar(255,0,0), 3 );
		}


	}






}
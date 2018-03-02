#include "feature_manager/feature_manager.h"

namespace holodeck {

	FeatureManager::FeatureManager()
	{

		k_ = 1.0;

		base_feature_matcher_ = std::make_shared<LK_Matcher>();

		// Used for display
		display_ = true;
		if (display_) {
			// cv::namedWindow("Moving Features", CV_WINDOW_NORMAL);
			// cv::resizeWindow("Moving Features", 1680/3, 1050/3);
			// cv::moveWindow("Moving Features",        0, 2*1050/3);

			cv::namedWindow("Optical Flow", CV_WINDOW_NORMAL);
			// cv::resizeWindow("Optical Flow", 1680/3, 1050/3);
			// cv::moveWindow("Optical Flow",        0, 1050/3);

		}
		

	}




	void FeatureManager::find_correspoinding_features(const cv::Mat& img)
	{

		// clear history
		prev_features_.clear();
		matched_features_.clear();

		base_feature_matcher_->find_correspoinding_features(img, prev_image_, prev_features_, matched_features_, mask_);
	
		calculate_pixel_velocity();

		// get_moving_pixel_velocity();

		if (display_)
			display(img);

		calculate_homography();

		prev_image_ = img.clone();
	}



	void FeatureManager::calculate_pixel_velocity() {

		pixel_velocity_.clear();

		for (int i = 0; i < prev_features_.size(); i++) {

			pixel_velocity_.push_back(matched_features_[i]-prev_features_[i]);
		
		}


	}

	void FeatureManager::calculate_homography() {


		if (matched_features_.size() > 5) {

			homography_ = cv::findHomography(prev_features_, matched_features_, cv::RANSAC);

		}


	}

	void FeatureManager::get_moving_pixel_velocity() {

		// Clear History
		moving_features_.clear();
		moving_velocity_.clear();

		// The normalized pixel velocities
		std::vector<float> norm_pixel_velocities;

		// Keeps track of original index
		std::vector<unsigned> index;

		// The mean
		float mean = 0;

		// The variance
		float var = 0;

		// The standard diviation
		float stdd = 0;

		// Normalize the velocities and finds the mean
		for (unsigned i = 0; i < pixel_velocity_.size(); i++) {

			// Calculate the norm of the velocity
			float norm = std::sqrt(std::pow(pixel_velocity_[i].x,2)+ std::pow(pixel_velocity_[i].y,2));
			
			// Ensure it is not an outlier
			if (fabs(norm) < 100) {

				// Push norm into the vector
				norm_pixel_velocities.push_back(norm);

				// Push index value onto index vector
				index.push_back(i);

				// std::cout << "norm: " << norm << std::endl;

				// Add to the mean
				mean +=norm;
			}

		}

		// Make sure there is at least one velocity measurement
		if (norm_pixel_velocities.size() > 0) {

			// Find the mean by dividing by the total count
			mean = mean / norm_pixel_velocities.size();

			// Find the variance
			for (int i = 0; i < norm_pixel_velocities.size(); i++) {
				var += pow(mean-norm_pixel_velocities[i],2);
			}
			var = var/norm_pixel_velocities.size();

			// Find the standard deviation
			stdd = std::sqrt(var);

		}

		// Find the velocities above a certain standard deviation
		for (int i = 0; i < norm_pixel_velocities.size(); i++) {

			// If the pixel velocity is about a certaind standard 
			// deviation, then assume the feature is moving
			if (norm_pixel_velocities[i] > (mean + stdd*k_)) {

				moving_features_.push_back(matched_features_[index[i]]);
				moving_velocity_.push_back(pixel_velocity_[index[i]]*frame_rate_);

			}

		}



	}

	void FeatureManager::display(const cv::Mat& img) {


		////////////////////// Display Features /////////////////////

		cv::Mat alteredImg = img.clone();

		// // add circles to the detected features
		// for( int i = 0; i < moving_features_.size(); i++) {

		// 		cv::circle(alteredImg,
		// 		 moving_features_[i],5, 
		// 		 cv::Scalar(255,0,0), 3 );
		// }

		// cv::imshow("Moving Features", alteredImg);
		// cv::waitKey(1);



		////////////////////////// Optical Flow //////////////////

		// alteredImg = img.clone();

		// add circles to the detected features
		for ( int i = 0; i < matched_features_.size(); i++) {

				cv::arrowedLine(alteredImg,
				 prev_features_[i],
				 prev_features_[i] + pixel_velocity_[i]*10, 
				 cv::Scalar(255,0,0), 3 );
		}

		cv::imshow("Optical Flow", alteredImg);
		cv::waitKey(1);

	}



}
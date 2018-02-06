#pragma once

#include "holodeck_controller/controller_manager/controller_base.h"

namespace holodeck {

	class ControllerOptFlow : public ControllerBase {

	public:

		ControllerOptFlow();


		//This method receives an image and states, and computes commands

		void implement_controller(const cv::Mat& img, const ros_holodeck::state state, float *command );

	private:

		// gftt algorithm
		cv::Ptr<cv::GFTTDetector> gftt_;

		cv::Mat prev_img_; // the previous image

		std::vector<cv::Point2f> prev_features_; // Features found by Gftt
		std::vector<cv::Point2f> next_features_; // Features matched by OpticalFlowLK
		std::vector<cv::Point2f> pixel_velocity_; // Pixel velocity

		bool first_image_ = true;


		// detect features on image
		void detect_features(const cv::Mat& img);

		// compute the optical flow and keep only the good features
		void compute_optical_flow(const cv::Mat& img);

		void compute_pixel_velocity();

		void display_image(const cv::Mat& img, float *command);

		void calculate_commands(const cv::Mat& img, float *command);

	};

}
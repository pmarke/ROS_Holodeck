#pragma once

#include <ros_holodeck/state.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>




namespace holodeck {

	class VisualOdometry {

	public:

		VisualOdometry();

		void implement_visual_odometry(const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features,  const ros_holodeck::state state); 

	private:

		// Parameters
		float frame_rate_;

		float image_width_ = 512; // Image width in pixels
		float image_height_ = 512; // Image height in pixels
		float focal_length_ = image_width_/2;

		// Rotation, and Translation matrices
		Eigen::Matrix<float,3,1> T_delta_;
		Eigen::Matrix<float,3,3> R_delta_;

		// State
		Eigen::Matrix<float,3,1> position_;
		Eigen::Matrix<float,3,3> attitude_;
		bool state_init_;

		// Map of where the drone has traveled
		cv::Mat map_;
		bool display_;

		// Rotation
		Eigen::Matrix<float,3,3> rot_cam2world_;

		// Calculate the Essential Matrix, Relative Rotation and Translation from features
		void calculate_relative_pose(const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features,  const ros_holodeck::state state); 

		// Update the sate
		void update_state(const ros_holodeck::state state);

		void display();

		void draw_map();


	};




}
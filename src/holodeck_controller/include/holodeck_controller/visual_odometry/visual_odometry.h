#pragma once

#include <ros_holodeck/state.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>




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

		// Homogenous matrix between frames
		Eigen::Matrix<float,4,4> G_delta_;

		// Homogenous matrix from world to current frame
		Eigen::Matrix<float,4,4> G_w2c_;
		bool state_init_;

		// Map of where the drone has traveled
		cv::Mat map_;
		bool display_;
		cv::Point2f pos_dl_; // Previous position


		// Calculate the Essential Matrix, Relative Rotation and Translation from features
		void calculate_relative_pose(const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features,  const ros_holodeck::state state); 

		// Update the sate
		void update_state(const ros_holodeck::state state);

		void display(const ros_holodeck::state state);

		// Draw the new position on the map
		void draw_map(const ros_holodeck::state state);


	};




}
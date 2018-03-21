#pragma once

#include <ros_holodeck/state.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <cmath>




namespace holodeck {

	class VisualOdometry {

	public:

		VisualOdometry();

		void implement_visual_odometry(const cv::Mat& proj_matrix, const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features,  const ros_holodeck::state state); 

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
		Eigen::MatrixXf G_w2c_prev_;
		bool state_init_;
		bool prev_init_;

		// Map of where the drone has traveled
		cv::Mat map_;
		bool display_;
		bool use_truth_;
		cv::Point2f pos_dl_; // Previous position

		cv::Mat points_3d_;
		std::vector<Eigen::Vector4f> X_vec_;
		

		Eigen::Matrix3f E_;

		// obtacle parameters
		int min_height_;
		int max_distance_;
		double fov_;
		int life_;


		int max_value_;
		cv::Mat obstacles_;
		cv::Mat decrease_ ;



		// Calculate the Essential Matrix, Relative Rotation and Translation from features
		void calculate_relative_pose(const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features,  const ros_holodeck::state state); 

		// Reconstruct the 3d point in the world frame. 
		void reconstruct_3d(const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features);

		// Reconstructs 3d world coordinate according to HartleyZisserman00-Chapter 12 Structure Computation
		// P is the camera matrix (K*[R|T] where K is the intrinsic parameters of the camera)
		// x is the homogenous pixel coordinates of the point X s.t x = [x, y, 1]^T
		// X the world homogenous coordinates of a point s.t. X = [X, Y, Z, 1]
		void reconstruct_3d_HZ_point(const Eigen::Matrix<float,3,4>& P1, const Eigen::Matrix<float,3,4>& P2, const Eigen::Vector2f& x1, const Eigen::Vector2f& x2, Eigen::Vector4f& X);

		void reconstruct_3d_HZ(const std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_features);

		// Minimize the reprojection error according to HartleyZisserman00-Chapter 12 Structure Computation
		void minimize_reprojection_error(const Eigen::Matrix3f& E, Eigen::Vector2f& x1, Eigen::Vector2f& x2);

		// Reconstructs the essential matrix from the two homo geneous transformations
		void reconstruct_essential_mat(const Eigen::Matrix<float,4,4>& G_w2c_prev, const Eigen::Matrix<float,4,4>& G_w2c, Eigen::Matrix3f& E);

		// Update the sate
		void update_state(const ros_holodeck::state state);

		// determines which new obstacle points are valid
		void handle_obstacle_points(const ros_holodeck::state state, const std::vector<Eigen::Vector4f>& new_obstacles);

		// Updates valid obstacle points
		void update_obstacle_points(const std::vector<Eigen::Vector4f>& new_obstacles);

		void display(const ros_holodeck::state state);

		// Draw the new position on the map
		void draw_map(const ros_holodeck::state state);


	};




}
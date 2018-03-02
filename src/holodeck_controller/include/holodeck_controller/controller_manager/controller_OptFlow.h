#pragma once


#include "controller_manager/controller_base.h"

namespace holodeck {


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


	enum ControllerStates {STOP, AVOID_OBSTACLE, CORRIDOR_BALANCE, MOVE_FORWARD, USER_CONTROL};

	// This class computes the optical flow in different regions in order
	// to navigate the coptor down the corridor and avoid obstacals. 
	class ControllerOptFlow : public ControllerBase {

	public:

		ControllerOptFlow();


		//This method receives an image and states, and computes commands

		void implement_controller(const cv::Mat& img, const ros_holodeck::state state, float *command );

	private:

		// gftt algorithm
		cv::Ptr<cv::GFTTDetector> gftt_;

		cv::Mat prev_img_;      // the previous image
		float image_width_ = 512; // Image width in pixels
		float image_height_ = 512; // Image height in pixels
		float focal_length_ = image_width_/2;
		float focal_length_2 = powf(focal_length_,2);
		bool first_image_ = true;
		float frames_per_second_ = 30;

		struct RegionOfInterest {
			std::vector<cv::Point2f> points;          // Image points were optical flow is being computed
			std::vector<cv::Point2f> good_points;    // The original points that have a successful match
			std::vector<cv::Point2f> matched_points;  // Matched points on second image found with optical flow
			std::vector<cv::Point2f> pixel_velocity;  // The optical flow
			int x_start, x_stop;                      // The x-direction ROI of the image 
			int y_start, y_stop;                      // The y-direction ROI of the image
			float count;                              // Number of good points matched
			float sum, avg;                           // The total and average optical flow in a ROI
		} left_wall_, right_wall_, left_center_, right_center_, bottom_, center_;

		float diff_wall_opt_flow_ = 0;
		float diff_center_opt_flow_ = 0;  
		float altitude_avg_ = 0;          // The height at which the coptor is flying  
		float time_till_collision_avg_ = 0;
		bool ttc_valid_ = false;

		ControllerStates controller_state_ = USER_CONTROL;
		// ControllerStates prev_state_ = 10;
		bool can_change_states_ = false;


		int state_timer_ = 0;




		// detect features on image
		// void detect_features(const cv::Mat& img);

		// compute the optical flow and keep only the good features
		void compute_optical_flow(const cv::Mat& img, const ros_holodeck::state state);

		// Find corresponding points between the previous image and new image
		void find_correspoinding_points(const cv::Mat& img,RegionOfInterest* roi);

		// compute pixel velocity between good_points and matched_points
		void compute_pixel_velocity(RegionOfInterest* roi, const ros_holodeck::state state);

		// Display optical Flow
		void display_image(const cv::Mat& img, float *command);

		// Draw the optical flow vectors on the image
		void draw_optical_flow(cv::Mat& img, RegionOfInterest* roi);

		// Find the average optical flow in a region
		void compute_avg_optical_flow(RegionOfInterest* roi);

		// Calculate the commands using optical flow
		void calculate_commands(const cv::Mat& img, const ros_holodeck::state state,float *command);

		// Calculate the height of the copter using optical flow
		void calculate_height(const ros_holodeck::state state);

		// Estimates time till collision
		void estimate_time_till_collision();

		// init prev_features_ with an even distribution inside the image plane
		void init_points(RegionOfInterest* roi);

	};

}
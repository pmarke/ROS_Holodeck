#pragma once


#include <opencv2/opencv.hpp>
#include <vector>
#include <ros_holodeck/state.h>
#include <ros_holodeck/command.h>
#include <stdlib.h> 
#include <math.h>


namespace holodeck {

	// This class serves as the base of all controllers
	class ControllerBase
	{
	public:

		//This method receives an image and states, and computes commands
		virtual void implement_controller(const cv::Mat& img, const ros_holodeck::state state, float *command) = 0;
	

	
	};



}
#pragma once

// ros
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ros_holodeck/state.h>
#include <ros_holodeck/state_srv.h>
#include <ros_holodeck/command.h>



#include <holodeck_cpp_bindings/holodeck.h>
#include <string>
#include <Eigen/Dense>
#include <math.h>

namespace holodeck {

    class ROSHolodeck {

    public:
        ROSHolodeck();

    private:

        // ros communication
        ros::Subscriber command_sub_;
        ros::ServiceServer state_srv_;
        image_transport::Publisher pub_video_;

        // name of the Holodeck environment
        std::string environment_name_;

        holodeck::Holodeck holodeck;


        ros_holodeck::state state_;

        // variables to hold state information and image
        Eigen::Matrix3f rotation_matrix_;  
        Eigen::Matrix<float, 6,1> imu_;
        Eigen::RowVector3f location_;
        Eigen::RowVector3f velocity_;
        cv::Mat img_;

        //ros time
        

        // callback when a new command is received
        void command_callback(const ros_holodeck::command& msg);

        // publish states: location, veclocity, imu, and rotation matrix
        bool service_state(ros_holodeck::state_srv::Request& req, ros_holodeck::state_srv::Response& res);

        // publish camera image captured by the Holodeck Agent
        void publish_video();
    };




}
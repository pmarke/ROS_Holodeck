#include "ros_holodeck/ros_holodeck.h"

namespace holodeck {

ROSHolodeck::ROSHolodeck() {

	// create private node handle
	ros::NodeHandle nh_("holodeck");

	// command callback
	command_sub_ = nh_.subscribe("command", 10, &ROSHolodeck::command_callback, this);

	// state publisher
	state_pub_ = nh_.advertise<ros_holodeck::state>("state",10);

	// image publisher
	image_transport::ImageTransport it(nh_);
	pub_video_ = it.advertise("image",10);

	nh_.param<std::string>("environment_name", environment_name_, "UrbanCity");

	// create the world
	holodeck.make(environment_name_, holodeck::OPENGL4);

}


void ROSHolodeck::command_callback(const ros_holodeck::command& msg) {

	// reset the environment
	if (msg.reset)
		holodeck.reset();

	// take a step in the world. ts = 1/30 (s)
	holodeck.step(-msg.roll, msg.pitch, -msg.altitude, msg.yaw_rate);

	// get states and image
	holodeck.get_orientation_sensor_data(rotation_matrix_);
	holodeck.get_imu_sensor_data(imu_);
	holodeck.get_location_sensor_data(location_);
	holodeck.get_velocity_sensor_data(velocity_);
	holodeck.get_primary_player_camera(img_);


	// publish state and video information
	publish_state();

	publish_video(); 



}



void ROSHolodeck::publish_state() {

	///
	// pack state information into ros msg
	//

	// rotation matrix
	for (int i= 0; i < 9; i++) 
		state_.R[i] = rotation_matrix_(i/3,i%3);  

	// imu
	state_.imu.gyro_x  = imu_(0);
	state_.imu.gyro_y  = imu_(1);
	state_.imu.gyro_z  = imu_(2);
	state_.imu.accel_x = imu_(3);
	state_.imu.accel_y = imu_(4);
	state_.imu.accel_z = imu_(5);

	// position
	state_.position.x = location_(0);
	state_.position.y = location_(1);
	state_.position.z = location_(2);

	// velocity
	state_.velocity.x = velocity_(0);
	state_.velocity.y = velocity_(1);
	state_.velocity.z = velocity_(2);

	// time
	state_.header.stamp = ros::Time::now();


	// publish state information
	state_pub_.publish(state_);

}



void ROSHolodeck::publish_video() {

	// publish the altered video
	sensor_msgs::ImagePtr msg;
	msg = cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::BGRA8, img_).toImageMsg();
	pub_video_.publish(msg);

}





}
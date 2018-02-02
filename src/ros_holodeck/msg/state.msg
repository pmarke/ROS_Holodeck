# State message


Header header

# in world frame
geometry_msgs/Vector3 velocity   # m/s
geometry_msgs/Vector3 position   # m

# in vehicle 1 frame
geometry_msgs/Vector3 vel        # m/s   # velocity in vehicle 1 frame

# imu data
ros_holodeck/imu imu

# Rotation Matrix body to world
float32[9] R

# Euler angles in radians
float32 roll
float32 pitch
float32 yaw

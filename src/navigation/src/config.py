# Params
MODE              = "SIL"
FRAME_MAP         = "map"
FRAME_BASE        = "turtlebot/kobuki/base_footprint"
FRAME_PREDICTED_BASE    = "turtlebot/kobuki/predicted_base_footprint"
FRAME_DEPTH_CAMERA_SIL  = "turtlebot/kobuki/realsense_depth"
FRAME_DEPTH_CAMERA_HIL  = "camera_depth_optical_frame"

PUB_KEYFRAME_DEADRECKONING_TOPIC = "/keyframes_deadReckoning"
PUB_ODOM_TOPIC          = "/odom"

SUB_GROUND_TRUTH_TOPIC  = "/turtlebot/kobuki/odom_ground_truth"
SUB_IMU_TOPIC           = "/turtlebot/kobuki/sensors/imu_data"
SUB_ODOM_TOPIC          = "/turtlebot/joint_states"

SERVICE_RESET_FILTER    = "ResetFilter"

ROBOT_WHEEL_BASE        = 0.235
ROBOT_WHEEL_RADIUS      = 0.035

STD_ODOM_X_VELOCITY     = 0.01          # [m/s]
STD_ODOM_Y_VELOCITY     = 0.001         # [m/s]
STD_ODOM_ROTATE_VELOCITY= 0.1           # [deg/s]
STD_MAG_HEADING         = 1             # [deg]


#
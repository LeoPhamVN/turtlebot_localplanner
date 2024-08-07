# Params
FRAME_MAP         = "map"
FRAME_BASE        = "turtlebot/kobuki/base_footprint"
# FRAME_PREDICTED_BASE    = "turtlebot/kobuki/predicted_base_footprint"

PUB_KEYFRAME_DEADRECKONING_TOPIC = "/keyframes_deadReckoning"
PUB_ODOM_TOPIC          = "/state_estimation"

SUB_GROUND_TRUTH_TOPIC  = "/turtlebot/kobuki/odom_ground_truth"
SUB_IMU_TOPIC           = "/turtlebot/kobuki/sensors/imu_data"
SUB_ODOM_TOPIC          = "/turtlebot/joint_states"

SERVICE_RESET_FILTER    = "ResetFilter"

ROBOT_WHEEL_BASE        = 0.235
ROBOT_WHEEL_RADIUS      = 0.035

STD_ODOM_X_VELOCITY     = 0.05          # [m/s]
STD_ODOM_Y_VELOCITY     = 0.005         # [m/s]
STD_ODOM_ROTATE_VELOCITY= 0.5           # [deg/s]
STD_MAG_HEADING         = 0.05          # [deg]


#
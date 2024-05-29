#!/usr/bin/python
import tf
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose as PoseMsg
from nav_msgs.msg import Odometry
from utils.EKF_3DOF_InputDisplacement_Heading import *
from utils.Odometry import *
from utils.Magnetometer import *
# from geometry_msgs.msg import PoseStamped

# from turtlebot_graph_slam.srv import ResetFilter, ResetFilterResponse
# from .config import *

class EKF:
    def __init__(self) -> None:
        # Init using sensors
        Qk          = np.diag(np.array([STD_ODOM_X_VELOCITY ** 2, STD_ODOM_Y_VELOCITY ** 2, np.deg2rad(STD_ODOM_ROTATE_VELOCITY) ** 2])) 
        Rk          = np.diag([np.deg2rad(STD_MAG_HEADING)**2])

        self.current_pose           = None
        self.xk           = np.zeros((3, 1))        # Robot pose in the k frame
        self.Pk           = np.zeros((3, 3))        # Robot covariance in the k frame  
        self.yawOffset    = 0.0
        self.ekf_filter   = None
        self.x_map        = np.zeros((3, 1))        # Robot pose in the map frame
        self.x_frame_k    = np.zeros((3, 1))
        self.mode         = MODE

        # PUBLISHERS   
        # Publisher for sending Odometry
        self.odom_pub           = rospy.Publisher(PUB_ODOM_TOPIC, Odometry, queue_size=1)

        # Publisher for sending Path
        # self.path_pub           = rospy.Publisher(PUB_PATH_TOPIC, Odometry, queue_size=1)
        
        # SUBSCRIBERS
        self.odom_sub               = rospy.Subscriber(SUB_ODOM_TOPIC, JointState, self.get_odom) 

        if self.mode == "SIL":
            self.ground_truth_sub   = rospy.Subscriber(SUB_GROUND_TRUTH_TOPIC, Odometry, self.get_ground_truth) 
        elif self.mode == "HIL":
            self.IMU_sub            = rospy.Subscriber(SUB_IMU_TOPIC, Imu, self.get_IMU) 

        self.odom   = OdomData(Qk)
        self.mag    = Magnetometer(Rk)

        # self.path   = Path()
        # self.path.header.frame_id = FRAME_MAP
        # if self.mode == "SIL":
        # Move
        while True:
            if self.current_pose is not None:
                self.yawOffset    = self.current_pose[2]
                break
        
        # SERVICES
    
        # TIMERS
        # rospy.Timer(rospy.Duration(1.0), self.pathPub)

        # Init EKF Filter
        self.ekf_filter = EKF_3DOF_InputDisplacement_Heading(self.xk, self.Pk, self.odom, self.mag)
    
    # Ground Truth callback: Gets current robot pose and stores it into self.current_pose. Besides, get heading as a measurement to update filter
    def get_ground_truth(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                            odom.pose.pose.orientation.y,
                                                            odom.pose.pose.orientation.z,
                                                            odom.pose.pose.orientation.w])
        
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])

        # Get heading as a measurement to update filter
        if self.mag.read_magnetometer(yaw-self.yawOffset) and self.ekf_filter is not None:
            self.ekf_filter.gotNewHeadingData()
    
    # IMU callback: Gets current robot orientation and stores it into self.current_pose. Besides, get heading as a measurement to update filter
    def get_IMU(self, Imu):
        _, _, yaw = tf.transformations.euler_from_quaternion([Imu.orientation.x, 
                                                            Imu.orientation.y,
                                                            Imu.orientation.z,
                                                            Imu.orientation.w])
        
        yaw = -yaw      # Imu in the turtlebot is NEU while I'm using NED

        self.current_pose = np.array([0.0, 0.0, yaw])

        # Get heading as a measurement to update filter
        if self.mag.read_magnetometer(yaw-self.yawOffset) and self.ekf_filter is not None:
            self.ekf_filter.gotNewHeadingData()

    # Odometry callback: Gets encoder reading to compute displacement of the robot as input of the EKF Filter.
    # Run EKF Filter with frequency of odometry reading
    def get_odom(self, odom):

        timestamp        = odom.header.stamp

        # Read encoder
        if (self.mode == "HIL" and len(odom.name) == 2) or self.mode == "SIL":
            if self.odom.read_encoder(odom) and self.ekf_filter is not None:
                self.ekf_filter.gotNewEncoderData()

        if self.ekf_filter is not None:
            # Run EKF Filter
            self.xk, self.Pk = self.ekf_filter.Localize(self.xk, self.Pk)

            self.x_map       = Pose3D.oplus(self.x_frame_k, self.xk)

            # Plot path
            # pose = PoseStamped()
            # pose.header = odom.header
            # pose.pose = odom.pose.pose

            # self.path.poses.append(pose)
            # self.path.header.stamp = rospy.Time.now()
            # self.path_pub.publish(self.path)

            # Publish rviz
            self.odom_path_pub(timestamp)             # Use for Localizarion only

            self.publish_tf_map(timestamp)

            if self.mode == "HIL":
                self.publish_tf_cam(timestamp)

    # Reset state and covariance of th EKF filter
    def reset_filter(self, request):
        self.yawOffset    += self.xk[2]
        self.xk           = np.zeros((3, 1))
        self.Pk           = np.zeros((3, 3))

        self.x_frame_k    = self.x_map

        # return ResetFilterResponse(request.reset_filter_requested)
    
    # Publish Filter results
    def odom_path_pub(self, timestamp):
        # Transform theta from euler to quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, float((self.xk[2, 0])))  # Convert euler angles to quaternion

        # Publish predicted odom
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = FRAME_MAP
        odom.child_frame_id = FRAME_PREDICTED_BASE


        odom.pose.pose.position.x = self.xk[0]
        odom.pose.pose.position.y = self.xk[1]

        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.pose.covariance = list(np.array([[self.Pk[0, 0], self.Pk[0, 1], 0, 0, 0, self.Pk[0, 2]],
                                [self.Pk[1, 0], self.Pk[1,1], 0, 0, 0, self.Pk[1, 2]],
                                [0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0],
                                [self.Pk[2, 0], self.Pk[2, 1], 0, 0, 0, self.Pk[2, 2]]]).flatten())

        # odom.twist.twist.linear.x = self.v
        # odom.twist.twist.angular.z = self.w

        self.odom_pub.publish(odom)

        tf.TransformBroadcaster().sendTransform((float(self.xk[0, 0]), float(self.xk[1, 0]), 0.0), quaternion, timestamp, odom.child_frame_id, odom.header.frame_id)
            

    def publish_tf_map(self, timestamp):
        x_map = self.x_map.copy()

        # Define the translation and rotation for the inverse TF (base_footprint to world)
        translation = (x_map[0], x_map[1], 0) # Set the x, y, z coordinates

        quaternion = tf.transformations.quaternion_from_euler(0, 0, x_map[2])  # Convert euler angles to quaternion
        rotation = (quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        
        # Publish the inverse TF from world to base_footprint
        tf.TransformBroadcaster().sendTransform(
            translation,
            rotation,
            timestamp,
            FRAME_BASE,
            FRAME_MAP
        )

    def publish_tf_cam(self, timestamp):
        # Define the translation and rotation for the inverse TF (base_footprint to world)
        translation = (0, 0, 0) # Set the x, y, z coordinates

        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)  # Convert euler angles to quaternion
        rotation = (quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        
        # Publish the inverse TF from world to base_footprint
        tf.TransformBroadcaster().sendTransform(
            translation,
            rotation,
            timestamp,
            "realsense_link",
            "camera_link"         
        )

    def spin(self):
        pass

if __name__ == '__main__':
    rospy.init_node('EKF_node')
    node = EKF()	
    
    rospy.spin()
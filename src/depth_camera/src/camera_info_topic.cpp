#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

class CameraInfoTopicNode
{
public:
    CameraInfoTopicNode()
    {
        // Initialize the ROS node
        ros::NodeHandle nh;

        // Subscribe to the camera info topic
        camera_info_sub_ = nh.subscribe("/turtlebot/kobuki/realsense/depth/camera_info", 1, &CameraInfoTopicNode::cameraInfoCallback, this);
        depth_camera_sub_sim_ = nh.subscribe("/turtlebot/kobuki/realsense/depth/image_depth", 1, &CameraInfoTopicNode::depthImgCallback, this);
        depth_camera_sub_real_ = nh.subscribe("/turtlebot/kobuki/realsense/depth/image_rect_raw", 1, &CameraInfoTopicNode::depthImgCallback, this);

        // Publish the camera info with a custom timestamp
        camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("/turtlebot/kobuki/realsense/depth/custom_camera_info", 1);
        depth_image_pub_ = nh.advertise<sensor_msgs::Image>("/turtlebot/kobuki/realsense/depth/custom_depth_image", 1);
        

        ros::Timer timer = nh.createTimer(ros::Duration(0.1), &CameraInfoTopicNode::timerCallback, this);
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        // Create a new camera info message with the same data
        custom_camera_info_ = *msg;

        last_time_ = msg->header.stamp;

        // camera_info_sub_.shutdown();
    }

    void timerCallback(const ros::TimerEvent& event)
    {
        // Update the timestamp of the camera info message
        custom_camera_info_.header.stamp = last_time_;

        // Publish the camera info message
        camera_info_pub_.publish(custom_camera_info_);
    }

    void depthImgCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        // Create a new camera info message with the same data
        custom_camera_info_.header.stamp = last_time_;

        // Publish the camera info message
        camera_info_pub_.publish(custom_camera_info_);

        depth_image_msg_ = *msg;
        depth_image_msg_.header.stamp = last_time_;

        // Publish the camera depth message
        depth_image_pub_.publish(depth_image_msg_);
    }

private:
    ros::Subscriber camera_info_sub_;
    ros::Subscriber depth_camera_sub_sim_;
    ros::Subscriber depth_camera_sub_real_;
    ros::Publisher camera_info_pub_;
    ros::Publisher depth_image_pub_;
    sensor_msgs::CameraInfo custom_camera_info_;
    ros::Time last_time_;
    ros::Timer timer;
    sensor_msgs::Image depth_image_msg_;
};

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "camera_info_topic_node");

    // Create an instance of the CameraInfoTopicNode class
    CameraInfoTopicNode node;

    // Spin the ROS node
    ros::spin();

    return 0;
}
/**
 * @file sensor_scan_generation.cpp
 * @author Leo Pham (phamthanhloc.bkhn@gmail.com)
 * @brief Node synchronize laser scan and odometry. Convert laser scan (in laser (camera) frame to map frame).
 * @version 0.1
 * @date 2024-05-19
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCLoudInMapFrame(new pcl::PointCloud<pcl::PointXYZ>());

double robotX = 0;
double robotY = 0;
double robotZ = 0;
double roll   = 0;
double pitch  = 0;
double yaw    = 0;

bool newTransformToMap = false;

nav_msgs::Odometry odometryIn;
tf::StampedTransform transformToMap;
tf::StampedTransform transformToMapOut;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;

ros::Publisher *pubOdometryPointer = NULL;
ros::Publisher pubLaserCloud;

string sub_odometry_topic_;
string sub_sensor_registered_scan_topic_;
string pub_state_estimation_at_scan_topic_;
string pub_registered_scan_topic_;

string frame_map_;
string frame_base_footprint_;
string frame_sensor_;
string frame_sensor_at_scan_;
string frame_vehicle_;
string frame_camera_;
string frame_depth_camera_;

double kSensorOffsetX;      // [met]
double kSensorOffsetY;      // [met]
double kSensorOffsetZ;      // [met]

double kSensorDirX;
double kSensorDirY;
double kSensorDirZ;

string kSensorType; 

void OdometryHandler(const nav_msgs::Odometry::ConstPtr& odometry)
{
  odometryIn = *odometry;

  transformToMap.setOrigin(
      tf::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, -0.05));
  transformToMap.setRotation(tf::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
                                            odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));
  
  // Invert the transformation
  tf::Transform transform = transformToMap.inverse();

  // Send TF "sensor"
  transformToMap.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  transformToMap.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  transformToMap.stamp_           = odometry->header.stamp;
  transformToMap.frame_id_        = frame_base_footprint_;
  transformToMap.child_frame_id_  = frame_sensor_;
  tfBroadcasterPointer->sendTransform(transformToMap);

  // Send TF "vehicle"
  transformToMap.child_frame_id_  = frame_vehicle_;
  tfBroadcasterPointer->sendTransform(transformToMap);

  // Send TF "camera"
  transformToMap.frame_id_        = frame_depth_camera_;
  transformToMap.child_frame_id_  = frame_camera_;
  tfBroadcasterPointer->sendTransform(transformToMap);
}

void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odometry,
                                  const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  laserCloudIn->clear();
  laserCLoudInMapFrame->clear();
  // float a = (odometry->header.stamp).toSec() - (laserCloud2->header.stamp).toSec();
  // std::cout<< a;
  pcl::fromROSMsg(*laserCloud2, *laserCloudIn);

  odometryIn = *odometry;

  transformToMap.setOrigin(
      tf::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
  transformToMap.setRotation(tf::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
                                            odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));

  int laserCloudInNum = laserCloudIn->points.size();

  pcl::PointXYZ p1;
  tf::Vector3 vec;

  // Transform from camera frame to "vehicle" frame
  for (int i = 0; i < laserCloudInNum; i++)
  {
    p1 = laserCloudIn->points[i];
    if (kSensorType == "depthCamera")
    {
      // Use for depth camera
      vec.setX(p1.z + kSensorOffsetX);
      vec.setY(p1.x + kSensorOffsetY);
      vec.setZ(p1.y + kSensorOffsetZ);
    }
    else if (kSensorType == "lidar")
    {
      // Use for Lidar
      vec.setX(kSensorDirX*p1.x + kSensorOffsetX);
      vec.setY(kSensorDirY*p1.y + kSensorOffsetY);
      vec.setZ(kSensorDirZ*p1.z + kSensorOffsetZ);
    }
    
    vec = transformToMap * vec;

    p1.x = vec.x();
    p1.y = vec.y();
    p1.z = vec.z();

    laserCLoudInMapFrame->points.push_back(p1);
  }

  // Publish odometry at scan
  odometryIn.header.stamp       = laserCloud2->header.stamp;
  odometryIn.header.frame_id    = frame_map_;
  odometryIn.child_frame_id     = frame_sensor_at_scan_;
  pubOdometryPointer->publish(odometryIn);

  // Send TF frame sensor at scan
  transformToMap.stamp_         = laserCloud2->header.stamp;
  transformToMap.frame_id_      = frame_map_;
  transformToMap.child_frame_id_= frame_sensor_at_scan_;
  tfBroadcasterPointer->sendTransform(transformToMap);

  // Publish laser scan in "map" frame
  sensor_msgs::PointCloud2 scan_data;
  pcl::toROSMsg(*laserCLoudInMapFrame, scan_data);
  scan_data.header.stamp        = laserCloud2->header.stamp;
  scan_data.header.frame_id     = frame_map_;
  pubLaserCloud.publish(scan_data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_scan");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("frame_map_", frame_map_);
  nhPrivate.getParam("frame_base_footprint_", frame_base_footprint_);
  nhPrivate.getParam("frame_sensor_", frame_sensor_);
  nhPrivate.getParam("frame_sensor_at_scan_", frame_sensor_at_scan_);
  nhPrivate.getParam("frame_vehicle_", frame_vehicle_);
  nhPrivate.getParam("frame_camera_", frame_camera_);
  nhPrivate.getParam("frame_depth_camera_", frame_depth_camera_);

  nhPrivate.getParam("sub_odometry_topic_", sub_odometry_topic_);
  nhPrivate.getParam("sub_sensor_registered_scan_topic_", sub_sensor_registered_scan_topic_);
  
  nhPrivate.getParam("pub_state_estimation_at_scan_topic_", pub_state_estimation_at_scan_topic_);
  nhPrivate.getParam("pub_registered_scan_topic_", pub_registered_scan_topic_);
  
  nhPrivate.getParam("kSensorOffsetX", kSensorOffsetX);
  nhPrivate.getParam("kSensorOffsetY", kSensorOffsetY);
  nhPrivate.getParam("kSensorOffsetZ", kSensorOffsetZ);

  nhPrivate.getParam("kSensorDirX", kSensorDirX);
  nhPrivate.getParam("kSensorDirY", kSensorDirY);
  nhPrivate.getParam("kSensorDirZ", kSensorDirZ);

  nhPrivate.getParam("kSensorType", kSensorType);


  // ROS message filters
  message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
  message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  // SUBCRIBE
  subOdometry.subscribe(nh, sub_odometry_topic_, 1);

  subLaserCloud.subscribe(nh, sub_sensor_registered_scan_topic_, 1);

  sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
  // sync_->setMaxIntervalDuration(ros::Duration(0.05)); // 50 milliseconds tolerance
  sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2));

  subOdometry.registerCallback(boost::bind(OdometryHandler, _1));

  // PUBLISH
  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> (pub_state_estimation_at_scan_topic_, 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>(pub_registered_scan_topic_, 2);

  ros::spin();

  return 0;
}

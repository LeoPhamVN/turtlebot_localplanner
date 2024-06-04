#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

double kSensorOffsetX = 0;
double kSensorOffsetY = 0;
int kPubSkipNum = 1;
int pubSkipCount = 0;
bool kTwoWayDrive = true;
double kLookAheadDis = 0.5;
double kYawRateGain = 7.5;
double kStopYawRateGain = 7.5;
double kMaxYawRate = 45.0;
double kMaxSpeed = 1.0;
double kMaxAccel = 1.0;
double kSwitchTimeThre = 1.0;
double kDirDiffThre = 0.1;
double kStopDisThre = 0.2;
double kSlowDwnDisThre = 1.0;
bool kUseInclRateToSlow = false;
double kInclRateThre = 120.0;
double kSlowRate1 = 0.25;
double kSlowRate2 = 0.5;
double kSlowTime1 = 2.0;
double kSlowTime2 = 2.0;
bool kUseInclToStop = false;
double kInclThre = 45.0;
double kStopTime = 5.0;
bool kNoRotAtStop = false;
bool kNoRotAtGoal = true;
bool kAutonomyMode = false;
double kAutonomySpeed = 1.0;
double kJoyToSpeedDelay = 2.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;

string sub_odometry_topic_;
string sub_path_topic_;
string sub_joystick_topic_;
string sub_speed_topic_;
string sub_stop_topic_;
 
string pub_speed_topic_;

nav_msgs::Path path;

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * kSensorOffsetX + sin(yaw) * kSensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * kSensorOffsetX - cos(yaw) * kSensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > kInclThre * PI / 180.0 || fabs(pitch) > kInclThre * PI / 180.0) && kUseInclToStop) {
    stopInitTime = odomIn->header.stamp.toSec();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > kInclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > kInclRateThre * PI / 180.0) && kUseInclRateToSlow) {
    slowInitTime = odomIn->header.stamp.toSec();
  }
}

void pathHandler(const nav_msgs::Path::ConstPtr& pathIn)
{
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  joyTime = ros::Time::now().toSec();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && kNoRotAtStop) joyYaw = 0;

  if (joy->axes[4] < 0 && !kTwoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  if (joy->axes[2] > -0.1) {
    kAutonomyMode = false;
  } else {
    kAutonomyMode = true;
  }
}

void speedHandler(const std_msgs::Float32::ConstPtr& speed)
{
  double speedTime = ros::Time::now().toSec();

  if (kAutonomyMode && speedTime - joyTime > kJoyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / kMaxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::Int8::ConstPtr& stop)
{
  safetyStop = stop->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathFollower");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("kSensorOffsetX", kSensorOffsetX);
  nhPrivate.getParam("kSensorOffsetY", kSensorOffsetY);
  nhPrivate.getParam("kPubSkipNum", kPubSkipNum);
  nhPrivate.getParam("kTwoWayDrive", kTwoWayDrive);
  nhPrivate.getParam("kLookAheadDis", kLookAheadDis);
  nhPrivate.getParam("kYawRateGain", kYawRateGain);
  nhPrivate.getParam("kStopYawRateGain", kStopYawRateGain);
  nhPrivate.getParam("kMaxYawRate", kMaxYawRate);
  nhPrivate.getParam("kMaxSpeed", kMaxSpeed);
  nhPrivate.getParam("kMaxAccel", kMaxAccel);
  nhPrivate.getParam("kSwitchTimeThre", kSwitchTimeThre);
  nhPrivate.getParam("kDirDiffThre", kDirDiffThre);
  nhPrivate.getParam("kStopDisThre", kStopDisThre);
  nhPrivate.getParam("kSlowDwnDisThre", kSlowDwnDisThre);
  nhPrivate.getParam("kUseInclRateToSlow", kUseInclRateToSlow);
  nhPrivate.getParam("kInclRateThre", kInclRateThre);
  nhPrivate.getParam("kSlowRate1", kSlowRate1);
  nhPrivate.getParam("kSlowRate2", kSlowRate2);
  nhPrivate.getParam("kSlowTime1", kSlowTime1);
  nhPrivate.getParam("kSlowTime2", kSlowTime2);
  nhPrivate.getParam("kUseInclToStop", kUseInclToStop);
  nhPrivate.getParam("kInclThre", kInclThre);
  nhPrivate.getParam("kStopTime", kStopTime);
  nhPrivate.getParam("kNoRotAtStop", kNoRotAtStop);
  nhPrivate.getParam("kNoRotAtGoal", kNoRotAtGoal);
  nhPrivate.getParam("kAutonomyMode", kAutonomyMode);
  nhPrivate.getParam("kAutonomySpeed", kAutonomySpeed);
  nhPrivate.getParam("kJoyToSpeedDelay", kJoyToSpeedDelay);

  nhPrivate.getParam("sub_odometry_topic_", sub_odometry_topic_);
  nhPrivate.getParam("sub_path_topic_", sub_path_topic_);
  nhPrivate.getParam("sub_joystick_topic_", sub_joystick_topic_);
  nhPrivate.getParam("sub_speed_topic_", sub_speed_topic_);
  nhPrivate.getParam("sub_stop_topic_", sub_stop_topic_);

  nhPrivate.getParam("pub_speed_topic_", pub_speed_topic_);

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> (sub_odometry_topic_, 5, odomHandler);

  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path> (sub_path_topic_, 5, pathHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> (sub_joystick_topic_, 5, joystickHandler);

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> (sub_speed_topic_, 5, speedHandler);

  ros::Subscriber subStop = nh.subscribe<std_msgs::Int8> (sub_stop_topic_, 5, stopHandler);

  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped> (pub_speed_topic_, 5);
  geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "map";

  if (kAutonomyMode) {
    joySpeed = kAutonomySpeed / kMaxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (pathInit) {
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < kLookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      if (kTwoWayDrive) {
        double time = ros::Time::now().toSec();
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > kSwitchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > kSwitchTimeThre) {
          navFwd = true;
          switchTime = time;
        }
      }

      float joySpeed2 = kMaxSpeed * joySpeed;
      if (!navFwd) {
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }

      if (fabs(vehicleSpeed) < 2.0 * kMaxAccel / 100.0) vehicleYawRate = -kStopYawRateGain * dirDiff;
      else vehicleYawRate = -kYawRateGain * dirDiff;

      if (vehicleYawRate > kMaxYawRate * PI / 180.0) vehicleYawRate = kMaxYawRate * PI / 180.0;
      else if (vehicleYawRate < -kMaxYawRate * PI / 180.0) vehicleYawRate = -kMaxYawRate * PI / 180.0;

      if (joySpeed2 == 0 && !kAutonomyMode) {
        vehicleYawRate = kMaxYawRate * joyYaw * PI / 180.0;
      } else if (pathSize <= 1 || (dis < kStopDisThre && kNoRotAtGoal)) {
        vehicleYawRate = 0;
      }

      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if (endDis / kSlowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / kSlowDwnDisThre;
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + kSlowTime1 && slowInitTime > 0) joySpeed3 *= kSlowRate1;
      else if (odomTime < slowInitTime + kSlowTime1 + kSlowTime2 && slowInitTime > 0) joySpeed3 *= kSlowRate2;

      if (fabs(dirDiff) < kDirDiffThre && dis > kStopDisThre) {
        if (vehicleSpeed < joySpeed3) vehicleSpeed += kMaxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= kMaxAccel / 100.0;
      } else {
        if (vehicleSpeed > 0) vehicleSpeed -= kMaxAccel / 100.0;
        else if (vehicleSpeed < 0) vehicleSpeed += kMaxAccel / 100.0;
      }

      if (odomTime < stopInitTime + kStopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if (safetyStop >= 1) vehicleSpeed = 0;
      if (safetyStop >= 2) vehicleYawRate = 0;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = ros::Time().fromSec(odomTime);
        if (fabs(vehicleSpeed) <= kMaxAccel / 100.0) cmd_vel.twist.linear.x = 0;
        else cmd_vel.twist.linear.x = vehicleSpeed;

        // if (vehicleYawRate < 0.7 && vehicleYawRate > 0.1) 
        // {
        //   vehicleYawRate = 0.7;
        // }
        // else if (vehicleYawRate < -0.1 && vehicleYawRate > -0.7)
        // {
        //   vehicleYawRate = -0.5;
        // }
        
        cmd_vel.twist.angular.z = vehicleYawRate;
        pubSpeed.publish(cmd_vel);

        pubSkipCount = kPubSkipNum;
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}

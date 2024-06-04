#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>

#include <laser_geometry/laser_geometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

// Third parties
#include <utils/pointcloud_utils.h>

using namespace std;

const double PI = 3.1415926;

#define PLOTPATHSET 1

string kPathFolder;
double kVehicleLength = 0.6;
double kVehicleWidth = 0.6;
double kSensorOffsetX = 0;
double kSensorOffsetY = 0;
double kSensorOffsetZ = 0;
bool kTwoWayDrive = true;
double kLaserVoxelSize = 0.05;
double kTerrainVoxelSize = 0.2;
bool kUseTerrainAnalysis = false;
bool kCheckObstacle = true;
bool kCheckRotObstacle = false;
double kAdjacentRange = 2.0;
double kObstacleHeightThre = 0.2;
double kGroundHeightThre = 0.1;
double kCostHeightThre = 0.1;
double kCostScore = 0.02;
bool kUseCost = false;
const int laserCloudStackNum = 1;
int laserCloudCount = 0;
int kPointPerPathThre = 2;
double kMinRelZ = -0.5;
double kMaxRelZ = 0.25;
double kMaxSpeed = 1.0;
double kDirWeight = 0.02;
double kDirThre = 90.0;
bool dirToVehicle = false;
double pathScale = 1.0;
double kMinPathScale = 0.75;
double kPathScaleStep = 0.25;
bool kPathScaleBySpeed = true;
double kMinPathRange = 1.0;
double kPathRangeStep = 0.5;
bool kPathRangeBySpeed = true;
bool kPathCropByGoal = true;
bool kAutonomyMode = false;
double kAutonomySpeed = 1.0;
double kJoyToSpeedDelay = 2.0;
double kJoyTokCheckObstacleDelay = 5.0;
double kGoalClearRange = 0.5;
double kGoalX = 3.0;
double kGoalY = 0.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyDir = 0;

const int pathNum = 343;
const int groupNum = 7;
float gridVoxelSize = 0.02;
float searchRadius = 0.45;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
const int gridVoxelNumX = 161;
const int gridVoxelNumY = 451;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

string sub_start_exploration_topic_;
string sub_odometry_topic_;
string sub_registered_scan_topic_;
string sub_terrain_map_topic_;
string sub_terrain_map_ext_topic_;
string sub_joystick_topic_;
string sub_waypoint_topic_;
string sub_waypoint_rviz_topic_;
string sub_speed_topic_;
string sub_navigation_boundary_topic_;
string sub_added_obstacles_topic_;
string sub_check_obstacle_topic_;

string pub_path_topic_;
string pub_free_path_topic_;
string pub_vertical_obstacles_topic_;
string pub_goal_topic_;

string kSensorType; 

double kSensorDirX;
double kSensorDirY;
double kSensorDirZ;

string kMode;

// typedef pcl::PointXYZRGBNormal PlannerCloudPointType;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZ>());
// pcl::PointCloud<pcl::PointXYZI>::Ptr vertical_surface_cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr goalCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
#endif

// pointcloud_utils_ns::VerticalSurfaceExtractor vertical_surface_extractor;

// pointcloud_utils_ns::PointCloudDownsizer<pcl::PointXYZ> pointcloud_downsizer_;
// std::unique_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> registered_cloud_;
// std::unique_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> vertical_surface_cloud_;

int pathList[pathNum] = {0};
float endDirPathList[pathNum] = {0};
int clearPathList[36 * pathNum] = {0};
float pathPenaltyList[36 * pathNum] = {0};
float clearPathPerGroupScore[36 * groupNum] = {0};
std::vector<int> correspondences[gridVoxelNum];

bool newLaserCloud = false;
bool newTerrainCloud = false;

double odomTime = 0;
double joyTime = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter;

laser_geometry::LaserProjection projector;


// Test
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCLoudInMapFrame(new pcl::PointCloud<pcl::PointXYZ>());
tf::StampedTransform transformToMap;
nav_msgs::Odometry odometryIn;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;

ros::Publisher *pubOdometryPointer = NULL;
ros::Publisher pubLaserCloud;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  odomTime = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x - cos(yaw) * kSensorOffsetX + sin(yaw) * kSensorOffsetY;
  vehicleY = odom->pose.pose.position.y - sin(yaw) * kSensorOffsetX - cos(yaw) * kSensorOffsetY;
  vehicleZ = odom->pose.pose.position.z;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  if (!kUseTerrainAnalysis) {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::fromROSMsg(*laserCloud2, *laserCloud);

    // pointcloud_downsizer_.Downsize(laserCloud, 0.2, 0.2, 0.2);
    // registered_cloud_->cloud_->clear();
    // pcl::copyPointCloud(*laserCloud, *(registered_cloud_->cloud_));
    // vertical_surface_cloud_->cloud_->clear();

    // vertical_surface_extractor.ExtractVerticalSurface<PlannerCloudPointType, PlannerCloudPointType>(
    //       registered_cloud_->cloud_, vertical_surface_cloud_->cloud_);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr vertical_surface_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::copyPointCloud(*(vertical_surface_cloud_->cloud_), *vertical_surface_cloud);

    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point.x = laserCloud->points[i].x;
      point.y = laserCloud->points[i].y;
      point.z = laserCloud->points[i].z;

      float pointX;
      float pointY;
      float pointZ;

      // Convert orientation from camera frame to vehicle frame
      if (kSensorType == "depthCamera")
      {
        pointX = point.z;
        pointY = point.x;
        pointZ = point.y;

      }
      else if (kSensorType == "lidar")
      {
        pointX = point.x;
        pointY = point.y;
        pointZ = point.z;
      }
      
      float dis = sqrt(pointX * pointX + pointY * pointY);
      if (dis < kAdjacentRange) {
        point.x = pointX + kSensorOffsetX;
        point.y = pointY + kSensorOffsetY;
        point.z = pointZ + kSensorOffsetZ;
        laserCloudCrop->push_back(point);
      }
    }

    laserCloudDwz->clear();
    laserDwzFilter.setInputCloud(laserCloudCrop);
    laserDwzFilter.filter(*laserCloudDwz);
    
    newLaserCloud = true;
  }
}

void laser2DCloudHandler(const sensor_msgs::LaserScanConstPtr& input)
{
  if (!kUseTerrainAnalysis) {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::fromROSMsg(*laserCloud2, *laserCloud);

    // pointcloud_downsizer_.Downsize(laserCloud, 0.2, 0.2, 0.2);
    // registered_cloud_->cloud_->clear();
    // pcl::copyPointCloud(*laserCloud, *(registered_cloud_->cloud_));
    // vertical_surface_cloud_->cloud_->clear();

    // vertical_surface_extractor.ExtractVerticalSurface<PlannerCloudPointType, PlannerCloudPointType>(
    //       registered_cloud_->cloud_, vertical_surface_cloud_->cloud_);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr vertical_surface_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::copyPointCloud(*(vertical_surface_cloud_->cloud_), *vertical_surface_cloud);

    // Convert LaserScan to PointCloud2
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*input, cloud);

    // Create a PCL point cloud from the PointCloud2 message
    laserCloud->clear();
    pcl::fromROSMsg(cloud, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point.x = laserCloud->points[i].x;
      point.y = laserCloud->points[i].y;
      point.z = laserCloud->points[i].z;

      float pointX;
      float pointY;
      float pointZ;

      // Convert orientation from camera frame to vehicle frame
      if (kSensorType == "depthCamera")
      {
        pointX = point.z;
        pointY = point.x;
        pointZ = point.y;

      }
      else if (kSensorType == "lidar")
      {
        pointX = kSensorDirX*point.x;
        pointY = kSensorDirY*point.y;
        pointZ = kSensorDirZ*point.z;
      }
      
      float dis = sqrt(pointX * pointX + pointY * pointY);
      if (dis < kAdjacentRange) {
        point.x = pointX + kSensorOffsetX;
        point.y = pointY + kSensorOffsetY;
        point.z = pointZ + kSensorOffsetZ;
        laserCloudCrop->push_back(point);
      }
    }

    laserCloudDwz->clear();
    laserDwzFilter.setInputCloud(laserCloudCrop);
    laserDwzFilter.filter(*laserCloudDwz);
    
    newLaserCloud = true;
  }
}

// void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odometry,
//                                   const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
// {
//   laserCloudIn->clear();
//   laserCLoudInMapFrame->clear();
//   laserCloudCrop->clear();

//   pcl::fromROSMsg(*laserCloud2, *laserCloudIn);

//   odometryIn = *odometry;

//   transformToMap.setOrigin(
//       tf::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
//   transformToMap.setRotation(tf::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
//                                             odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));

//   int laserCloudInNum = laserCloudIn->points.size();

//   pcl::PointXYZI p1;
//   pcl::PointXYZI p2;
//   tf::Vector3 vec;

//   pcl::PointXYZI point;
//   laserCloudCrop->clear();

//   // Transform from camera frame to "vehicle" frame
//   for (int i = 0; i < laserCloudInNum; i++)
//   {
//     point = laserCloudIn->points[i];
//     // Convert orientation from camera frame to vehicle frame
//     float pointX = point.z;
//     float pointY = point.x;
//     float pointZ = point.y;

//       float dis = sqrt(pointX * pointX + pointY * pointY);
//       if (dis < kAdjacentRange) {
//         point.x = pointX + kSensorOffsetX;
//         point.y = pointY + kSensorOffsetY;
//         point.z = pointZ + kSensorOffsetZ;
//         laserCloudCrop->push_back(point);
//       }

//     p1 = laserCloudIn->points[i];
//     vec.setX(p1.z + kSensorOffsetX);
//     vec.setY(p1.x + kSensorOffsetY);
//     vec.setZ(p1.y + kSensorOffsetZ);

    
//     vec = transformToMap * vec;

//     p1.x = vec.x();
//     p1.y = vec.y();
//     p1.z = vec.z();

//     laserCLoudInMapFrame->points.push_back(p1);
//   }

//   laserCloudDwz->clear();
//   laserDwzFilter.setInputCloud(laserCloudCrop);
//   laserDwzFilter.filter(*laserCloudDwz);

//   newLaserCloud = true;

//   // Publish laser scan in "map" frame
//   sensor_msgs::PointCloud2 scan_data;
//   pcl::toROSMsg(*laserCLoudInMapFrame, scan_data);
//   scan_data.header.stamp        = laserCloud2->header.stamp;
//   scan_data.header.frame_id     = "map";
//   pubLaserCloud.publish(scan_data);
// }



void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloud2)
{
  if (kUseTerrainAnalysis) {
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

    pcl::PointXYZI point;
    terrainCloudCrop->clear();
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < kAdjacentRange && (point.intensity > kObstacleHeightThre || kUseCost)) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        terrainCloudCrop->push_back(point);
      }
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);

    newTerrainCloud = true;
  }
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  joyTime = ros::Time::now().toSec();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;

  if (joySpeed > 0) {
    joyDir = atan2(joy->axes[3], joy->axes[4]) * 180 / PI;
    if (joy->axes[4] < 0) joyDir *= -1;
  }

  if (joy->axes[4] < 0 && !kTwoWayDrive) joySpeed = 0;

  if (joy->axes[2] > -0.1) {
    kAutonomyMode = false;
  } else {
    kAutonomyMode = true;
  }

  if (joy->axes[5] > -0.1) {
    kCheckObstacle = true;
  } else {
    kCheckObstacle = false;
  }
}

void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
  kGoalX = goal->point.x;
  kGoalY = goal->point.y;
  // kGoalX = goal->pose.position.x;
  // kGoalY = goal->pose.position.y;

  // goalCloud->clear();
  // pcl::PointXYZI point;
  // point.x = kGoalX;
  // point.y = kGoalY;
  // point.z = 0.0;
  // goalCloud->push_back(point);
}

void goalRvizHandler(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  kGoalX = goal->pose.position.x;
  kGoalY = goal->pose.position.y;

  goalCloud->clear();
  pcl::PointXYZI point;
  point.x = kGoalX;
  point.y = kGoalY;
  point.z = 0.0;
  goalCloud->push_back(point);
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


void boundaryHandler(const geometry_msgs::PolygonStamped::ConstPtr& boundary)
{
  boundaryCloud->clear();
  pcl::PointXYZI point, point1, point2;
  int boundarySize = boundary->polygon.points.size();

  if (boundarySize >= 1) {
    point2.x = boundary->polygon.points[0].x;
    point2.y = boundary->polygon.points[0].y;
    point2.z = boundary->polygon.points[0].z;
  }

  for (int i = 0; i < boundarySize; i++) {
    point1 = point2;

    point2.x = boundary->polygon.points[i].x;
    point2.y = boundary->polygon.points[i].y;
    point2.z = boundary->polygon.points[i].z;

    if (point1.z == point2.z) {
      float disX = point1.x - point2.x;
      float disY = point1.y - point2.y;
      float dis = sqrt(disX * disX + disY * disY);

      int pointNum = int(dis / kTerrainVoxelSize) + 1;
      for (int pointID = 0; pointID < pointNum; pointID++) {
        point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
        point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
        point.z = 0;
        point.intensity = 100.0;

        for (int j = 0; j < kPointPerPathThre; j++) {
          boundaryCloud->push_back(point);
        }
      }
    }
  }
}

void addedObstaclesHandler(const sensor_msgs::PointCloud2ConstPtr& addedObstacles2)
{
  addedObstacles->clear();
  pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

  int addedObstaclesSize = addedObstacles->points.size();
  for (int i = 0; i < addedObstaclesSize; i++) {
    addedObstacles->points[i].intensity = 200.0;
  }
}

void kCheckObstacleHandler(const std_msgs::Bool::ConstPtr& checkObs)
{
  double checkObsTime = ros::Time::now().toSec();

  if (kAutonomyMode && checkObsTime - joyTime > kJoyTokCheckObstacleDelay) {
    kCheckObstacle = checkObs->data;
  }
}

int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  return pointNum;
}

void readStartPaths()
{
  string fileName = kPathFolder + "/startPaths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

#if PLOTPATHSET == 1
void readPaths()
{
  string fileName = kPathFolder + "/paths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point;
  int pointSkipNum = 30;
  int pointSkipCount = 0;
  int val1, val2, val3, val4, val5, pathID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  fclose(filePtr);
}
#endif

void readPathList()
{
  string fileName = kPathFolder + "/pathList.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    printf ("\nIncorrect path number, exit.\n\n");
    exit(1);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
    }
  }

  fclose(filePtr);
}

void readCorrespondences()
{
  string fileName = kPathFolder + "/correspondences.txt";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int val1, gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    while (1) {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1) {
        printf ("\nError reading input files, exit.\n\n");
          exit(1);
      }

      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else {
        break;
      }
    }
  }

  fclose(filePtr);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("kPathFolder", kPathFolder);
  nhPrivate.getParam("kVehicleLength", kVehicleLength);
  nhPrivate.getParam("kVehicleWidth", kVehicleWidth);
  nhPrivate.getParam("kSensorOffsetX", kSensorOffsetX);
  nhPrivate.getParam("kSensorOffsetY", kSensorOffsetY);
  nhPrivate.getParam("kSensorOffsetZ", kSensorOffsetZ);
  nhPrivate.getParam("kTwoWayDrive", kTwoWayDrive);
  nhPrivate.getParam("kLaserVoxelSize", kLaserVoxelSize);
  nhPrivate.getParam("kTerrainVoxelSize", kTerrainVoxelSize);
  nhPrivate.getParam("kUseTerrainAnalysis", kUseTerrainAnalysis);
  nhPrivate.getParam("kCheckObstacle", kCheckObstacle);
  nhPrivate.getParam("kCheckRotObstacle", kCheckRotObstacle);
  nhPrivate.getParam("kAdjacentRange", kAdjacentRange);
  nhPrivate.getParam("kObstacleHeightThre", kObstacleHeightThre);
  nhPrivate.getParam("kGroundHeightThre", kGroundHeightThre);
  nhPrivate.getParam("kCostHeightThre", kCostHeightThre);
  nhPrivate.getParam("kCostScore", kCostScore);
  nhPrivate.getParam("kUseCost", kUseCost);
  nhPrivate.getParam("kPointPerPathThre", kPointPerPathThre);
  nhPrivate.getParam("kMinRelZ", kMinRelZ);
  nhPrivate.getParam("kMaxRelZ", kMaxRelZ);
  nhPrivate.getParam("kMaxSpeed", kMaxSpeed);
  nhPrivate.getParam("kDirWeight", kDirWeight);
  nhPrivate.getParam("kDirThre", kDirThre);
  nhPrivate.getParam("dirToVehicle", dirToVehicle);
  nhPrivate.getParam("pathScale", pathScale);
  nhPrivate.getParam("kMinPathScale", kMinPathScale);
  nhPrivate.getParam("kPathScaleStep", kPathScaleStep);
  nhPrivate.getParam("kPathScaleBySpeed", kPathScaleBySpeed);
  nhPrivate.getParam("kMinPathRange", kMinPathRange);
  nhPrivate.getParam("kPathRangeStep", kPathRangeStep);
  nhPrivate.getParam("kPathRangeBySpeed", kPathRangeBySpeed);
  nhPrivate.getParam("kPathCropByGoal", kPathCropByGoal);
  nhPrivate.getParam("kAutonomyMode", kAutonomyMode);
  nhPrivate.getParam("kAutonomySpeed", kAutonomySpeed);
  nhPrivate.getParam("kJoyToSpeedDelay", kJoyToSpeedDelay);
  nhPrivate.getParam("kJoyTokCheckObstacleDelay", kJoyTokCheckObstacleDelay);
  nhPrivate.getParam("kGoalClearRange", kGoalClearRange);
  nhPrivate.getParam("kGoalX", kGoalX);
  nhPrivate.getParam("kGoalY", kGoalY);
  nhPrivate.getParam("sub_start_exploration_topic_", sub_start_exploration_topic_);

  nhPrivate.getParam("sub_odometry_topic_", sub_odometry_topic_);
  nhPrivate.getParam("sub_registered_scan_topic_", sub_registered_scan_topic_);
  nhPrivate.getParam("sub_terrain_map_topic_", sub_terrain_map_topic_);
  nhPrivate.getParam("sub_terrain_map_ext_topic_", sub_terrain_map_ext_topic_);
  nhPrivate.getParam("sub_joystick_topic_", sub_joystick_topic_);
  nhPrivate.getParam("sub_waypoint_topic_", sub_waypoint_topic_);
  nhPrivate.getParam("sub_waypoint_rviz_topic_", sub_waypoint_rviz_topic_);
  nhPrivate.getParam("sub_speed_topic_", sub_speed_topic_);
  nhPrivate.getParam("sub_navigation_boundary_topic_", sub_navigation_boundary_topic_);
  nhPrivate.getParam("sub_added_obstacles_topic_", sub_added_obstacles_topic_);
  nhPrivate.getParam("sub_check_obstacle_topic_", sub_check_obstacle_topic_);

  nhPrivate.getParam("pub_path_topic_", pub_path_topic_);
  nhPrivate.getParam("pub_free_path_topic_", pub_free_path_topic_);
  nhPrivate.getParam("pub_vertical_obstacles_topic_", pub_vertical_obstacles_topic_);
  nhPrivate.getParam("pub_goal_topic_", pub_goal_topic_);

  nhPrivate.getParam("kSensorType", kSensorType);

  nhPrivate.getParam("Mode", kMode);

  if (kMode == "SIL")
  {
    nhPrivate.getParam("kSensorDirXSIL", kSensorDirX);
    nhPrivate.getParam("kSensorDirYSIL", kSensorDirY);
    nhPrivate.getParam("kSensorDirZSIL", kSensorDirZ);
  }
  else if (kMode == "HIL")
  {
    nhPrivate.getParam("kSensorDirXHIL", kSensorDirX);
    nhPrivate.getParam("kSensorDirYHIL", kSensorDirY);
    nhPrivate.getParam("kSensorDirZHIL", kSensorDirZ);
  }
  
  
  // Test
  // ROS message filters
  // message_filters::Subscriber<nav_msgs::Odometry> subOdometrySync;
  // message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
  // typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
  // typedef message_filters::Synchronizer<syncPolicy> Sync;
  // boost::shared_ptr<Sync> sync_;

  // // SUBCRIBE
  // subOdometrySync.subscribe(nh, sub_odometry_topic_, 1);

  // subLaserCloud.subscribe(nh, "/sensor_registered_scan", 1);

  // sync_.reset(new Sync(syncPolicy(100), subOdometrySync, subLaserCloud));
  // sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2));
  // 

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>
                                (sub_odometry_topic_, 5, odometryHandler);

  // ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
  //                                 (sub_registered_scan_topic_, 5, laserCloudHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::LaserScan>
                                  (sub_registered_scan_topic_, 5, laser2DCloudHandler);

  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                    (sub_terrain_map_topic_, 5, terrainCloudHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> (sub_joystick_topic_, 5, joystickHandler);

  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> (sub_waypoint_topic_, 5, goalHandler);

  ros::Subscriber subRvizGoal = nh.subscribe<geometry_msgs::PoseStamped> (sub_waypoint_rviz_topic_, 5, goalRvizHandler);

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> (sub_speed_topic_, 5, speedHandler);

  ros::Subscriber subBoundary = nh.subscribe<geometry_msgs::PolygonStamped> (sub_navigation_boundary_topic_, 5, boundaryHandler);

  ros::Subscriber subAddedObstacles = nh.subscribe<sensor_msgs::PointCloud2> (sub_added_obstacles_topic_, 5, addedObstaclesHandler);

  ros::Subscriber subkCheckObstacle = nh.subscribe<std_msgs::Bool> (sub_check_obstacle_topic_, 5, kCheckObstacleHandler);

  ros::Publisher pubPath = nh.advertise<nav_msgs::Path> (pub_path_topic_, 5);

  ros::Publisher pubVerticalPointCloud =  nh.advertise<sensor_msgs::PointCloud2>(pub_vertical_obstacles_topic_, 10);

  ros::Publisher pubGoalPointCloud =  nh.advertise<sensor_msgs::PointCloud2>(pub_goal_topic_, 10);

  nav_msgs::Path path;

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("test_pc", 2);

  #if PLOTPATHSET == 1
  ros::Publisher pubFreePaths = nh.advertise<sensor_msgs::PointCloud2> (pub_free_path_topic_, 2);
  #endif

  //ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/stacked_scans", 2);

  printf ("\nReading path files.\n");

  if (kAutonomyMode) {
    joySpeed = kAutonomySpeed / kMaxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  for (int i = 0; i < laserCloudStackNum; i++) {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  #if PLOTPATHSET == 1
  for (int i = 0; i < pathNum; i++) {
    paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  #endif
  for (int i = 0; i < gridVoxelNum; i++) {
    correspondences[i].resize(0);
  }

  laserDwzFilter.setLeafSize(kLaserVoxelSize, kLaserVoxelSize, kLaserVoxelSize);
  terrainDwzFilter.setLeafSize(kTerrainVoxelSize, kTerrainVoxelSize, kTerrainVoxelSize);

  readStartPaths();
  #if PLOTPATHSET == 1
  readPaths();
  #endif
  readPathList();
  readCorrespondences();

  printf ("\nInitialization complete.\n\n");

  ros::Rate rate(1);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newLaserCloud || newTerrainCloud) {
      if (newLaserCloud) {
        newLaserCloud = false;

        laserCloudStack[laserCloudCount]->clear();
        *laserCloudStack[laserCloudCount] = *laserCloudDwz;
        laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

        plannerCloud->clear();
        for (int i = 0; i < laserCloudStackNum; i++) {
          *plannerCloud += *laserCloudStack[i];
        }
      }

      if (newTerrainCloud) {
        newTerrainCloud = false;

        plannerCloud->clear();
        *plannerCloud = *terrainCloudDwz;
      }

      float sinVehicleRoll = sin(vehicleRoll);
      float cosVehicleRoll = cos(vehicleRoll);
      float sinVehiclePitch = sin(vehiclePitch);
      float cosVehiclePitch = cos(vehiclePitch);
      float sinVehicleYaw = sin(vehicleYaw);
      float cosVehicleYaw = cos(vehicleYaw);

      pcl::PointXYZI point;
      plannerCloudCrop->clear();
      int plannerCloudSize = plannerCloud->points.size();
      for (int i = 0; i < plannerCloudSize; i++) {
        point.x = plannerCloud->points[i].x;
        point.y = plannerCloud->points[i].y;
        point.z = plannerCloud->points[i].z;
        point.intensity = plannerCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < kAdjacentRange && ((point.z > kMinRelZ && point.z < kMaxRelZ) || kUseTerrainAnalysis)) {
          plannerCloudCrop->push_back(point);
        }
      }

      sensor_msgs::PointCloud2 pc_msg;
      pcl::toROSMsg(*plannerCloudCrop, pc_msg);
      pc_msg.header.stamp = ros::Time().fromSec(odomTime);
      pc_msg.header.frame_id = "vehicle";
      pubVerticalPointCloud.publish(pc_msg);

      sensor_msgs::PointCloud2 gpc_msg;
      pcl::toROSMsg(*goalCloud, gpc_msg);
      gpc_msg.header.stamp = ros::Time();
      gpc_msg.header.frame_id = "map";
      pubGoalPointCloud.publish(gpc_msg);
      

      int boundaryCloudSize = boundaryCloud->points.size();
      for (int i = 0; i < boundaryCloudSize; i++) {
        point.x = ((boundaryCloud->points[i].x - vehicleX) * cosVehicleYaw 
                + (boundaryCloud->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(boundaryCloud->points[i].x - vehicleX) * sinVehicleYaw 
                + (boundaryCloud->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = boundaryCloud->points[i].z;
        point.intensity = boundaryCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < kAdjacentRange) {
          plannerCloudCrop->push_back(point);
        }
      }

      int addedObstaclesSize = addedObstacles->points.size();
      for (int i = 0; i < addedObstaclesSize; i++) {
        point.x = ((addedObstacles->points[i].x - vehicleX) * cosVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(addedObstacles->points[i].x - vehicleX) * sinVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = addedObstacles->points[i].z;
        point.intensity = addedObstacles->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < kAdjacentRange) {
          plannerCloudCrop->push_back(point);
        }
      }

      float pathRange = kAdjacentRange;
      if (kPathRangeBySpeed) pathRange = kAdjacentRange * joySpeed;
      if (pathRange < kMinPathRange) pathRange = kMinPathRange;
      float relativeGoalDis = kAdjacentRange;

      if (kAutonomyMode) {
        float relativekGoalX = ((kGoalX - vehicleX) * cosVehicleYaw + (kGoalY - vehicleY) * sinVehicleYaw);
        float relativekGoalY = (-(kGoalX - vehicleX) * sinVehicleYaw + (kGoalY - vehicleY) * cosVehicleYaw);

        relativeGoalDis = sqrt(relativekGoalX * relativekGoalX + relativekGoalY * relativekGoalY);
        joyDir = atan2(relativekGoalY, relativekGoalX) * 180 / PI;

        if (!kTwoWayDrive) {
          if (joyDir > 90.0) joyDir = 90.0;
          else if (joyDir < -90.0) joyDir = -90.0;
        }
      }

      bool pathFound = false;
      float defPathScale = pathScale;
      if (kPathScaleBySpeed) pathScale = defPathScale * joySpeed;
      if (pathScale < kMinPathScale) pathScale = kMinPathScale;

      while (pathScale >= kMinPathScale && pathRange >= kMinPathRange) {
        for (int i = 0; i < 36 * pathNum; i++) {
          clearPathList[i] = 0;
          pathPenaltyList[i] = 0;
        }
        for (int i = 0; i < 36 * groupNum; i++) {
          clearPathPerGroupScore[i] = 0;
        }

        float minObsAngCW = -180.0;
        float minObsAngCCW = 180.0;
        float diameter = sqrt(kVehicleLength / 2.0 * kVehicleLength / 2.0 + kVehicleWidth / 2.0 * kVehicleWidth / 2.0);
        float angOffset = atan2(kVehicleWidth, kVehicleLength) * 180.0 / PI;
        int plannerCloudCropSize = plannerCloudCrop->points.size();
        for (int i = 0; i < plannerCloudCropSize; i++) {
          float x = plannerCloudCrop->points[i].x / pathScale;
          float y = plannerCloudCrop->points[i].y / pathScale;
          float h = plannerCloudCrop->points[i].intensity;
          float dis = sqrt(x * x + y * y);

          if (dis < pathRange / pathScale && (dis <= (relativeGoalDis + kGoalClearRange) / pathScale || !kPathCropByGoal) && kCheckObstacle) {
            for (int rotDir = 0; rotDir < 36; rotDir++) {
              float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
              float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
              if (angDiff > 180.0) {
                angDiff = 360.0 - angDiff;
              }
              if ((angDiff > kDirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > kDirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                  ((10.0 * rotDir > kDirThre && 360.0 - 10.0 * rotDir > kDirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
                continue;
              }

              float x2 = cos(rotAng) * x + sin(rotAng) * y;
              float y2 = -sin(rotAng) * x + cos(rotAng) * y;

              float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY 
                             * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

              int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
              int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
              if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {
                int ind = gridVoxelNumY * indX + indY;
                int blockedPathByVoxelNum = correspondences[ind].size();
                for (int j = 0; j < blockedPathByVoxelNum; j++) {
                  if (h > kObstacleHeightThre || !kUseTerrainAnalysis) {
                    clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                  } else {
                    if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h > kGroundHeightThre) {
                      pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                    }
                  }
                }
              }
            }
          }

          if (dis < diameter / pathScale && (fabs(x) > kVehicleLength / pathScale / 2.0 || fabs(y) > kVehicleWidth / pathScale / 2.0) && 
              (h > kObstacleHeightThre || !kUseTerrainAnalysis) && kCheckRotObstacle) {
            float angObs = atan2(y, x) * 180.0 / PI;
            if (angObs > 0) {
              if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
              if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = angObs + angOffset - 180.0;
            } else {
              if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
              if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = 180.0 + angObs - angOffset;
            }
          }
        }

        if (minObsAngCW > 0) minObsAngCW = 0;
        if (minObsAngCCW < 0) minObsAngCCW = 0;

        for (int i = 0; i < 36 * pathNum; i++) {
          int rotDir = int(i / pathNum);
          float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
          if (angDiff > 180.0) {
            angDiff = 360.0 - angDiff;
          }
          if ((angDiff > kDirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > kDirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
              ((10.0 * rotDir > kDirThre && 360.0 - 10.0 * rotDir > kDirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
            continue;
          }

          if (clearPathList[i] < kPointPerPathThre) {
            float penaltyScore = 1.0 - pathPenaltyList[i] / kCostHeightThre;
            if (penaltyScore < kCostScore) penaltyScore = kCostScore;

            float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0));
            if (dirDiff > 360.0) {
              dirDiff -= 360.0;
            }
            if (dirDiff > 180.0) {
              dirDiff = 360.0 - dirDiff;
            }

            float rotDirW;
            if (rotDir < 18) rotDirW = fabs(fabs(rotDir - 9) + 1);
            else rotDirW = fabs(fabs(rotDir - 27) + 1);
            float score = (1 - sqrt(sqrt(kDirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore;
            if (score > 0) {
              clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;
            }
          }
        }

        float maxScore = 0;
        int selectedGroupID = -1;
        for (int i = 0; i < 36 * groupNum; i++) {
          int rotDir = int(i / groupNum);
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
          float rotDeg = 10.0 * rotDir;
          if (rotDeg > 180.0) rotDeg -= 360.0;
          if (maxScore < clearPathPerGroupScore[i] && ((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || 
              (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && kTwoWayDrive) || !kCheckRotObstacle)) {
            maxScore = clearPathPerGroupScore[i];
            selectedGroupID = i;
          }
        }

        if (selectedGroupID >= 0) {
          int rotDir = int(selectedGroupID / groupNum);
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180;

          selectedGroupID = selectedGroupID % groupNum;
          int selectedPathLength = startPaths[selectedGroupID]->points.size();
          path.poses.resize(selectedPathLength);
          for (int i = 0; i < selectedPathLength; i++) {
            float x = startPaths[selectedGroupID]->points[i].x;
            float y = startPaths[selectedGroupID]->points[i].y;
            float z = startPaths[selectedGroupID]->points[i].z;
            float dis = sqrt(x * x + y * y);

            if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
              path.poses[i].pose.position.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
              path.poses[i].pose.position.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
              path.poses[i].pose.position.z = pathScale * z;
            } else {
              path.poses.resize(i);
              break;
            }
          }

          path.header.stamp = ros::Time().fromSec(odomTime);
          path.header.frame_id = "vehicle";
          pubPath.publish(path);

          #if PLOTPATHSET == 1
          freePaths->clear();
          for (int i = 0; i < 36 * pathNum; i++) {
            int rotDir = int(i / pathNum);
            float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
            float rotDeg = 10.0 * rotDir;
            if (rotDeg > 180.0) rotDeg -= 360.0;
            float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
            if (angDiff > 180.0) {
              angDiff = 360.0 - angDiff;
            }
            if ((angDiff > kDirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > kDirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                ((10.0 * rotDir > kDirThre && 360.0 - 10.0 * rotDir > kDirThre) && fabs(joyDir) > 90.0 && dirToVehicle) || 
                !((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || 
                (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && kTwoWayDrive) || !kCheckRotObstacle)) {
              continue;
            }

            if (clearPathList[i] < kPointPerPathThre) {
              int freePathLength = paths[i % pathNum]->points.size();
              for (int j = 0; j < freePathLength; j++) {
                point = paths[i % pathNum]->points[j];

                float x = point.x;
                float y = point.y;
                float z = point.z;

                float dis = sqrt(x * x + y * y);
                if (dis <= pathRange / pathScale && (dis <= (relativeGoalDis + kGoalClearRange) / pathScale || !kPathCropByGoal)) {
                  point.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                  point.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                  point.z = pathScale * z;
                  point.intensity = 1.0;

                  freePaths->push_back(point);
                }
              }
            }
          }

          sensor_msgs::PointCloud2 freePaths2;
          pcl::toROSMsg(*freePaths, freePaths2);
          freePaths2.header.stamp = ros::Time().fromSec(odomTime);
          freePaths2.header.frame_id = "vehicle";
          pubFreePaths.publish(freePaths2);
          #endif
        }

        if (selectedGroupID < 0) {
          if (pathScale >= kMinPathScale + kPathScaleStep) {
            pathScale -= kPathScaleStep;
            pathRange = kAdjacentRange * pathScale / defPathScale;
          } else {
            pathRange -= kPathRangeStep;
          }
        } else {
          pathFound = true;
          break;
        }
      }
      pathScale = defPathScale;

      if (!pathFound) {
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = ros::Time().fromSec(odomTime);
        path.header.frame_id = "vehicle";
        pubPath.publish(path);

        #if PLOTPATHSET == 1
        freePaths->clear();
        sensor_msgs::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths, freePaths2);
        freePaths2.header.stamp = ros::Time().fromSec(odomTime);
        freePaths2.header.frame_id = "vehicle";
        pubFreePaths.publish(freePaths2);
        #endif
      }

      /*sensor_msgs::PointCloud2 plannerCloud2;
      pcl::toROSMsg(*plannerCloudCrop, plannerCloud2);
      plannerCloud2.header.stamp = ros::Time().fromSec(odomTime);
      plannerCloud2.header.frame_id = "vehicle";
      pubLaserCloud.publish(plannerCloud2);*/
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}

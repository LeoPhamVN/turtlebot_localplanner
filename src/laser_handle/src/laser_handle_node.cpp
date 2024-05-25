#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <message_filters/subscriber.h>

using namespace std;

// Global node handle and publisher
laser_geometry::LaserProjection projector;
ros::Publisher pubLaserCloud;

float kSensorRange;
float kSensorVerticalAngle;
float kSensorHeight;
float kHeightExtendStep;
float kFloorExtendStep;

string laser_scan_topic_;
string pub_sensor_registered_scan_topic_;



int getSign(int num) {
    if (num > 0) {
        return 1; // Positive
    } else if (num < 0) {
        return -1; // Negative
    } else {
        return 0; // Zero
    }
}

struct Point {
    float x;
    float y;
};

std::vector<Point> getPointsBetween(const Point& p1, const Point& p2) {
    std::vector<Point> pointList;

    float dx = abs(p2.x - p1.x);
    float dy = abs(p2.y - p1.y);
    if (dx == 0 && dy == 0)
        return pointList;
    float sx = p1.x < p2.x ? kFloorExtendStep : -kFloorExtendStep;
    float sy = p1.y < p2.y ? kFloorExtendStep : -kFloorExtendStep;
    float err = dx - dy;

    float x = p1.x;
    float y = p1.y;

    while (true) {
        pointList.push_back({x, y});

        if (abs(x) >= abs(p2.x) || abs(y) >= abs(p2.y)) {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }

    return pointList;
}

void laserScanCallback(const sensor_msgs::LaserScanConstPtr& input)
{
    // Convert LaserScan to PointCloud2
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*input, cloud);

    // Create a PCL point cloud from the PointCloud2 message
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);

    // Create a new PCL point cloud to store the extended 3D points
    pcl::PointCloud<pcl::PointXYZ> extended_cloud;

    int rows = 2 * kSensorRange / kFloorExtendStep + 1;
    int cols = 2 * kSensorRange / kFloorExtendStep + 1;
    int arr[rows][cols] = {};
    Point p1 = {0, 0};

    // Iterate through the original point cloud
    for (const auto& point : pcl_cloud.points)
    {   
        float distance = sqrt(point.x * point.x + point.y * point.y);
        if (distance > kSensorRange)
            continue;

        // Create additional points at different z-values
        for (float z = kSensorHeight; z >= -0.4; z -= kHeightExtendStep)
        {
            pcl::PointXYZ extended_point;
            extended_point.x = point.x;
            extended_point.y = point.y;
            extended_point.z = z;
            float tilt_angle = atan2(extended_point.z, distance);
            if (abs(tilt_angle) < kSensorVerticalAngle)
                extended_cloud.points.push_back(extended_point);
        }

        Point p2 = {point.x, point.y};
        std::vector<Point> pointLists = getPointsBetween(p1, p2);

        for (const Point& p : pointLists) 
        {
            int num_x = int(p.x / kFloorExtendStep + rows/2);
            int num_y = int(p.y / kFloorExtendStep + cols/2);
            if (arr[num_x][num_y] != 1)
            {
                pcl::PointXYZ floor_point;
                floor_point.x = p.x;
                floor_point.y = p.y;
                floor_point.z = kSensorHeight;
                extended_cloud.points.push_back(floor_point);
                arr[num_x][num_y] = 1;
            }
        }

    }

    // Convert the extended PCL point cloud back to a ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(extended_cloud, output);
    output.header = cloud.header;

    // Publish the extended point cloud
    pubLaserCloud.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_handle");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("laser_scan_topic_", laser_scan_topic_);
    nhPrivate.getParam("pub_sensor_registered_scan_topic_", pub_sensor_registered_scan_topic_);
    nhPrivate.getParam("kSensorRange", kSensorRange);
    nhPrivate.getParam("kSensorVerticalAngle", kSensorVerticalAngle);
    nhPrivate.getParam("kSensorHeight", kSensorHeight);
    nhPrivate.getParam("kExtendStep", kHeightExtendStep);
    nhPrivate.getParam("kFloorExtendStep", kFloorExtendStep);   

    kSensorVerticalAngle *= M_PI/180.0;
    // SUBCRIBE
    message_filters::Subscriber<sensor_msgs::LaserScan> subLaser;
    subLaser.subscribe(nh, laser_scan_topic_, 1);
    subLaser.registerCallback(boost::bind(laserScanCallback, _1));

    // Initialize publisher
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>(pub_sensor_registered_scan_topic_, 1);

    ros::spin();

    return 0;
}
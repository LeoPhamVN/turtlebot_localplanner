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

class Grid {
public:
    // Constructor
    Grid(float resolution, float halfLengthX, float halfLengthY)
        : resolution(resolution), halfLengthX(halfLengthX), halfLengthY(halfLengthY) {
        if (resolution <= 0 || halfLengthX <= 0 || halfLengthY <= 0) {
            throw std::invalid_argument("Resolution and half lengths must be positive.");
        }
        int resolution_inv = static_cast<int>(1/resolution);
        // Calculate number of cells based on half lengths and resolution
        sizeX =  static_cast<int>((2 * halfLengthX) * resolution_inv);
        sizeY =  static_cast<int>((2 * halfLengthY) * resolution_inv);

        originX_sub = static_cast<int>((halfLengthX) * resolution_inv - 1);
        originY_sub = static_cast<int>((halfLengthY) * resolution_inv - 1);
        // Initialize grid with 0s
        grid.resize(sizeX, std::vector<int>(sizeY, 0));
        gridOut.resize(sizeX, std::vector<int>(sizeY, 0));
    }

    // Method to convert meters to grid coordinates
    std::pair<int, int> pos2Sub(double x, double y) const {
        int gridX = static_cast<int>((x + halfLengthX) / resolution);
        int gridY = static_cast<int>((y + halfLengthY) / resolution);

        if (gridX < 0 || gridX >= sizeX || gridY < 0 || gridY >= sizeY) {
            throw std::out_of_range("Coordinates are out of grid bounds.");
        }

        return {gridX, gridY};
    }

    // Method to convert grid coordinates to meters
    std::pair<double, double> sub2Pos(int gridX, int gridY) const {
        if (gridX < 0 || gridX >= sizeX || gridY < 0 || gridY >= sizeY) {
            throw std::out_of_range("Grid coordinates are out of grid bounds.");
        }

        double x = gridX * resolution - halfLengthX;
        double y = gridY * resolution - halfLengthY;

        return {x, y};
    }

    // Method to set a cell to 1
    void setCellOccupancy(double x, double y) {
        auto [gridX, gridY] = pos2Sub(x, y);
        grid[gridX][gridY] = 1;
    }

    // Method to get the value of a cell
    int getCellValue(double x, double y) const {
        auto [gridX, gridY] = pos2Sub(x, y);
        return grid[gridX][gridY];
    }

    // Method to get the value of a cell in Sub
    int getCellValueSub(int x, int y) const {
        return grid[x][y];
    }

    // Method to get the value of a cell in Sub
    int getCellValueSubOut(int x, int y) const {
        return gridOut[x][y];
    }

    bool checkVivibility(int gridX1, int gridY1) const {
        int gridX0 = originX_sub;
        int gridY0 = originY_sub;

        std::vector<std::pair<int, int>> cells;
        int dx = std::abs(gridX1 - gridX0);
        int dy = std::abs(gridY1 - gridY0);

        int sx = gridX0 < gridX1 ? 1 : -1;
        int sy = gridY0 < gridY1 ? 1 : -1;

        int err = dx - dy;

        while (true) { 
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                gridX0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                gridY0 += sy;
            }
            if (gridX0 == gridX1 && gridY0 == gridY1) break;
            if (grid[gridX0][gridY0] == 1 ||
                grid[gridX0+1][gridY0] == 1 ||
                grid[gridX0-1][gridY0] == 1 ||
                grid[gridX0][gridY0-1] == 1 ||
                grid[gridX0][gridY0-1] == 1)
                {
                    return false;
                }
        }
        return true;
    }

    void fillGrid()
    {
        for (int x = 0; x < sizeX; x++)
        {
            for (int y = 0; y < sizeY; y++)
            if (checkVivibility(x, y))
                gridOut[x][y] = 1;
        }
    }


    // Method to get list of cells on the line between two points
    std::vector<std::pair<int, int>> getCellsOnLine(double x0, double y0, double x1, double y1) const {
        auto [gridX0, gridY0] = pos2Sub(x0, y0);
        auto [gridX1, gridY1] = pos2Sub(x1, y1);

        std::vector<std::pair<int, int>> cells;
        int dx = std::abs(gridX1 - gridX0);
        int dy = std::abs(gridY1 - gridY0);

        int sx = gridX0 < gridX1 ? 1 : -1;
        int sy = gridY0 < gridY1 ? 1 : -1;

        int err = dx - dy;

        while (true) { 
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                gridX0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                gridY0 += sy;
            }
            if (gridX0 == gridX1 && gridY0 == gridY1) break;
            cells.emplace_back(gridX0, gridY0);
        }

        return cells;
    }

    // Method to set cells along rays from the center
    void setCellsAlongRays() {
        auto [centerX, centerY] = pos2Sub(0.0, 0.0);

        float angleStepRad = atan2(1, sizeX/2)*2;

        for (float angle = 0; angle < 2*M_PI; angle += angleStepRad) {
            double xEnd = (halfLengthX-resolution) * cos(angle);
            double yEnd = (halfLengthY-resolution) * sin(angle);

            auto cells = getCellsOnLine(0.0, 0.0, xEnd, yEnd);
            for (const auto& cell : cells) {
                if (grid[cell.first][cell.second] == 1 ||
                    grid[cell.first+1][cell.second] == 1 ||
                    grid[cell.first-1][cell.second] == 1 ||
                    grid[cell.first][cell.second-1] == 1 ||
                    grid[cell.first][cell.second-1] == 1) break;
                gridOut[cell.first][cell.second] = 1;
            }
        }
    }

    int getSizeX() {return sizeX;}
    int getSizeY() {return sizeY;}

private:
    float resolution; // Resolution of the grid in meters per cell
    float halfLengthX; // Half length of the grid in the x direction in meters
    float halfLengthY; // Half length of the grid in the y direction in meters
    int sizeX;         // Number of cells in the x direction
    int sizeY;         // Number of cells in the y direction
    int originX_sub;
    int originY_sub;
    std::vector<std::vector<int>> grid; // 2D vector to store grid values
    std::vector<std::vector<int>> gridOut; // 2D vector to store grid values
};

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

    Grid laserScan(kFloorExtendStep, kSensorRange, kSensorRange);

    int rows = 2 * kSensorRange / kFloorExtendStep + 1;
    int cols = 2 * kSensorRange / kFloorExtendStep + 1;
    int arr[rows][cols] = {};
    Point p1 = {0, 0};

    // Iterate through the original point cloud
    for (const auto& point : pcl_cloud.points)
    {   
        float distance = sqrt(point.x * point.x + point.y * point.y);
        if (distance > kSensorRange || distance < 0.2)
            continue;

        laserScan.setCellOccupancy(point.x, point.y);

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

        // Point p2 = {point.x, point.y};
        // std::vector<Point> pointLists = getPointsBetween(p1, p2);

        // for (const Point& p : pointLists) 
        // {
        //     int num_x = int(p.x / kFloorExtendStep + rows/2);
        //     int num_y = int(p.y / kFloorExtendStep + cols/2);
        //     if (arr[num_x][num_y] != 1)
        //     {
        //         pcl::PointXYZ floor_point;
        //         floor_point.x = p.x;
        //         floor_point.y = p.y;
        //         floor_point.z = kSensorHeight;
        //         extended_cloud.points.push_back(floor_point);
        //         arr[num_x][num_y] = 1;
        //     }
        // }

    }

    laserScan.fillGrid();
    for (int x = 0; x < laserScan.getSizeX(); ++x) {
        for (int y = 0; y < laserScan.getSizeY(); ++y) {
            if (laserScan.getCellValueSub(x, y) == 1 || laserScan.getCellValueSubOut(x, y) == 1 ) {
                auto [meterX, meterY] = laserScan.sub2Pos(x, y);
                extended_cloud.points.push_back(pcl::PointXYZ(meterX, meterY, kSensorHeight));
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

void laserScanCallback_slam(const sensor_msgs::PointCloud2& input)
{
    // Create a PCL point cloud from the PointCloud2 message
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(input, pcl_cloud);

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
        if (distance > kSensorRange || distance < 0.2)
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
    // output.header = cloud.header;

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
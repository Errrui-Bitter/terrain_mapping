/**
 * * * @description:
 * * * @filename: obstacle_mapping.h
 * * * @author: wangxurui
 * * * @date: 2025-03-14 18:56:22
 **/
#include <iostream>
#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> // For pcl::transformPointCloud
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

using namespace std;

class obstacle_mapping
{
public:
    obstacle_mapping(ros::NodeHandle &nh);
    ~obstacle_mapping();

private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher local_map_pub_;
    ros::Publisher local_fused_map_pub_;
    ros::Publisher local_global_map_pub_;
    ros::Publisher local_obstacle_map_pub_;
    tf::TransformListener listener_;
    ros::Time local_map_time_;

    grid_map::GridMap local_map_;
    grid_map::GridMap fused_local_map_;
    grid_map::GridMap global_map_;

    std::deque<nav_msgs::Odometry> odom_deque_;
    nav_msgs::Odometry cur_odom_;
    nav_msgs::Odometry last_odom_;

    string lidar_topic_;
    string map_frame_;
    string robot_frame_;
    string lidar_frame_;

    double minZ_;
    double maxZ_;
    double local_map_resolution_;
    double global_map_resolution_;
    double global_map_length_x_;
    double global_map_length_y_;
    double local_map_length_x_;
    double local_map_length_y_;
    double submap_length_x_;
    double submap_length_y_;
    double alpha_;
    double heightdiffthreshold_;
    double cntratiothreshold_;
    double lidar_z_;
    const double sensorRangeLimit_ = 60;
    const double predictionKernalSize_ = 2.0;

    int cntthreshold_;
    int mapping_mode_;

    bool first_map_init_ = true;
    bool use_EMA_;

    Eigen::Vector3d robot_pose_;

    void init();
    void initgridmap(grid_map::GridMap &map, string frame, double height_map_resolution, double maplength_x, double maplength_y);

    void StoreOdom(const nav_msgs::OdometryConstPtr &odom);

    void Mapping(const sensor_msgs::PointCloud2ConstPtr &msg);

    void LocalMapping(const pcl::PointCloud<pcl::PointXYZ> &localcloud);
    void FuseMap();
    void GlobalMapping(const pcl::PointCloud<pcl::PointXYZ> &globalcloud);

    void DenseMapping(grid_map::GridMap &map);
    void GradientMapping(grid_map::GridMap &map);
    void NormalMapping(grid_map::GridMap &map);
    void obstacledetection(grid_map::GridMap &map);
    void obstacledetection(const pcl::PointCloud<pcl::PointXYZ> &localcloud, grid_map::GridMap &map);

    void updateHeightStats(float &height, float &variance, float n, float new_height);

    int TimestampMatch(double fixtimestamp, std::deque<nav_msgs::Odometry> target_deque);
    double BayesUpdator(double value_update, double value_observe);

    void publocalmap();
    void pubfusedlocalmap();
    void pubglobalmap();
};

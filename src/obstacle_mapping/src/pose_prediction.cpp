/**
 * * * @description:
 * * * @filename: pose_prediction.cpp
 * * * @author: wangxurui
 * * * @date: 2025-04-07 17:51:04
 **/

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

struct HeightGrid
{
    Eigen::MatrixXd X_;
    Eigen::MatrixXd Y_;
    Eigen::MatrixXd Z_;
    Eigen::MatrixXd Gap_;
};

class pose_prediction
{
private:
    // 机器人参数
    // 底盘高度图位置
    std::string ROBOTFILENAME_ = ros::package::getPath("obstacle_mapping") + "/test_map/" + "6t_hight.csv";
    std::string TERRIANFILENAME_ = ros::package::getPath("obstacle_mapping") + "/test_map/" + "elevation1.csv";
    // 底盘高度图分辨率
    double RESOLUTION_ = 0.2;
    HeightGrid robot_gridmap_; // 机器人底盘高度图
    HeightGrid robot_gap_map_; // 全局坐标系下高度图和间隙图
    double d_touch_gap_ = 0.05;
    int rows_ = 16;
    int cols_ = 27;
    double length_ = RESOLUTION_ * 27;
    double wideth_ = RESOLUTION_ * 16;
    double center_x_ = wideth_ / 2;
    double center_y_ = length_ / 2;
    double center_z_ = 0.0;
    double local_min_z_;
    double lidar_z_;
    int interation_;

    std::vector<Eigen::Vector3d> touch_points_;
    std::vector<cv::Point2f> local_2D_touch_points_;
    std::vector<Eigen::Vector3d> touch_poly_;
    std::vector<cv::Point2f> local_2D_touch_poly_;
    std::array<Eigen::Vector2d, 4> foot_print_;
    Eigen::ParametrizedLine<double, 3> rotation_line_;

    Eigen::Vector2d pos_;
    Eigen::Vector3d T_vector_;
    Eigen::Matrix3d R_matrix_;
    Eigen::Matrix4d T_matrix_;
    Eigen::Matrix4d Rotate_with_rotation_line_;
    std::vector<cv::Point2f> touch_points_2D_;

    // 以车辆为中心的局部地图中提取的submap，认为这个submap中心点与车辆几何中心重合
    grid_map::GridMap terrain_sub_gridmap_;
    grid_map::Length submap_length_;
    grid_map::GridMap terrain_gridmap_; // 地形图
    HeightGrid terrain_gridmap_HG_;     // 地形图HG格式
    Eigen::MatrixXf terrainmatrix_;

    bool stable_;
    bool touch_points_change_;
    bool key_pushed_ = true;

    ros::NodeHandle nh_;
    ros::Timer maptimer_;
    ros::Subscriber map_sub_;
    ros::Subscriber key_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher terrain_map_pub_;
    ros::Publisher terrain_submap_pub_;
    ros::Publisher robot_gap_map_pub_;
    tf::TransformBroadcaster br_;

public:
    pose_prediction(ros::NodeHandle &nh);
    ~pose_prediction();
    void loadCsvToMatrix(const string &FILENAME, HeightGrid &map, int rows, int cols);
    void mapcallback(const grid_map_msgs::GridMap::ConstPtr msg);
    void KeyCallback(const std_msgs::Bool::ConstPtr key);
    void calRotationLine();
    bool ifInPoly(const Eigen::Vector3d &intersection);
    double calRotationAngle();
    void predict();
    void showPoseInfo();
    void showPose();
    void iterateOnce();
    void pubishmap(const ros::TimerEvent &event);
    void setCenterXYZ(HeightGrid &heightmap, const double &startX, const double &startY, const double &startZ);
    Eigen::ParametrizedLine<double, 3> getRotationLine(const Eigen::Vector3d &intersection, const Eigen::Vector3d &projection, const Eigen::Vector3d &F);
    Eigen::Vector3d fitplane(vector<Eigen::Vector3d> points);
    Eigen::Vector3d getProjectePoint(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &a);
};

pose_prediction::pose_prediction(ros::NodeHandle &nh) : nh_(nh)
{
    nh_.param<double>("query_pos_x", pos_.x(), 5.0);
    nh_.param<double>("query_pos_y", pos_.y(), 0.0);
    nh_.param<double>("lidar_z_", lidar_z_, 0.0);

    loadCsvToMatrix(ROBOTFILENAME_, robot_gridmap_, 16, 27);
    setCenterXYZ(robot_gridmap_, center_x_, center_y_, center_z_);
    loadCsvToMatrix(TERRIANFILENAME_, terrain_gridmap_HG_, 200, 300);
    setCenterXYZ(terrain_gridmap_HG_, 0, 0, -lidar_z_);
    grid_map::Length mapLength(40.0, 60.0);
    terrain_gridmap_.setFrameId("map");
    terrain_gridmap_.setGeometry(mapLength, 0.2, grid_map::Position(0.0, 0.0));
    terrain_gridmap_.add("elevation", terrain_gridmap_HG_.Z_.cast<float>());

    map_sub_ = nh_.subscribe<grid_map_msgs::GridMap>("/localmap", 10, boost::bind(&pose_prediction::mapcallback, this, _1));
    key_sub_ = nh_.subscribe<std_msgs::Bool>("/key", 10, boost::bind(&pose_prediction::KeyCallback, this, _1));
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/pose_visualization", 10);
    terrain_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/terrain_map", 10);
    terrain_submap_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/terrain_submap", 10);
    robot_gap_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/robot_map", 10);
    maptimer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&pose_prediction::pubishmap, this, _1));
}

pose_prediction::~pose_prediction()
{
}

void pose_prediction::loadCsvToMatrix(const string &FILENAME, HeightGrid &map, int rows, int cols)
{
    std::ifstream inputFile(FILENAME);
    if (!inputFile.is_open())
    {
        std::cerr << "Failed to open the file." << std::endl;
        return;
    }

    std::string line;
    std::string token;

    map.X_.resize(rows, cols);
    map.Y_.resize(rows, cols);
    map.Z_.resize(rows, cols);
    map.Gap_.resize(rows, cols);

    inputFile.clear();                 // 清除错误状态
    inputFile.seekg(0, std::ios::beg); // 将文件指针设置为文件的起始位置

    int rowIndex = 0;
    while (getline(inputFile, line))
    {
        int columnIndex = 0;
        std::istringstream tokenStream(line);
        std::string token;
        while (getline(tokenStream, token, ','))
        {
            map.X_(rowIndex, columnIndex) = rowIndex * RESOLUTION_ + RESOLUTION_ / 2;
            map.Y_(rowIndex, columnIndex) = columnIndex * RESOLUTION_ + RESOLUTION_ / 2;
            map.Z_(rowIndex, columnIndex) = std::stod(token) * RESOLUTION_ / 0.2;
            map.Gap_(rowIndex, columnIndex) = 0;
            ++columnIndex;
        }
        ++rowIndex;
    }

    inputFile.close();

    return;
}

void pose_prediction::setCenterXYZ(HeightGrid &heightmap, const double &startX, const double &startY, const double &startZ)
{
    for (auto x = heightmap.X_.data(); x < heightmap.X_.data() + heightmap.X_.size(); ++x)
    {
        *x -= startX;
        *x = -*x;
    }
    for (auto y = heightmap.Y_.data(); y < heightmap.Y_.data() + heightmap.Y_.size(); ++y)
    {
        *y -= startY;
        *y = -*y;
    }
    for (auto z = heightmap.Z_.data(); z < heightmap.Z_.data() + heightmap.Z_.size(); ++z)
    {
        *z -= startZ;
    }
}

void pose_prediction::mapcallback(const grid_map_msgs::GridMap::ConstPtr msg)
{
    grid_map::GridMapRosConverter::fromMessage(*msg, terrain_gridmap_);
}

/*!
 * 预测位姿
 * @param pos 需要估计的车辆局部坐标系下的查询位置
 * @return:
 */
void pose_prediction::predict()
{
    ROS_INFO("Predicting Pose on X:%.2f, Y:%.2f ......", pos_.x(), pos_.y());
    touch_points_.clear();
    local_2D_touch_points_.clear();
    touch_poly_.clear();
    local_2D_touch_poly_.clear();

    stable_ = false;
    touch_points_change_ = true;

    double radius = sqrt(wideth_ * wideth_ + length_ * length_);
    submap_length_ = grid_map::Length(radius, radius);
    // 获取查询位置的子图
    bool success_submap = false;
    terrain_sub_gridmap_ = terrain_gridmap_.getSubmap(pos_, submap_length_, success_submap);
    if (success_submap == false)
    {
        ROS_WARN("cound not get full submap!");
        return;
    }
    /****************************
    先根据法向量估计大致姿态
    ****************************/
    vector<Eigen::Vector3d> terrain_map_points;
    for (grid_map::GridMapIterator it(terrain_sub_gridmap_); !it.isPastEnd(); ++it)
    {
        Eigen::Vector3d point;
        if (terrain_sub_gridmap_.getPosition3("elevation", *it, point))
        {
            terrain_map_points.push_back(point);
        }
    }
    Eigen::Vector3d normal_vec = fitplane(terrain_map_points);

    Eigen::Matrix3d rotationMatrix;

    Eigen::Vector3d zAxis(0, 0, 1);
    Eigen::Vector3d rotationAxis = zAxis.cross(normal_vec);
    double angle = acos(zAxis.dot(normal_vec));

    rotationMatrix = Eigen::AngleAxisd(angle, rotationAxis.normalized()).toRotationMatrix();
    T_matrix_ = Eigen::Matrix4d::Zero();
    T_matrix_.block<3, 3>(0, 0) = rotationMatrix;
    T_matrix_.block<4, 1>(0, 3) = Eigen::Vector4d(pos_(0), pos_(1), 0.0, 1.0);

    // debug
    std::ofstream outfile("/home/wxr/proj/traversability/obstacle_detection/ws_obdt/src/obstacle_mapping/poses/T_matrix_init.txt");
    if (outfile.is_open())
    {
        outfile << T_matrix_ << std::endl;
        outfile.close();
        std::cout << "Matrix exported to T_matrix_init.txt" << std::endl;
    }
    else
    {
        std::cerr << "Error opening file for writing." << std::endl;
    }

    /****************************
    迭代初始化
    ****************************/
    robot_gap_map_ = robot_gridmap_;
    terrainmatrix_ = terrain_sub_gridmap_["elevation"];
    double min_gap = 100;
    double min_local_z = 0;

    // 找到最小间距
    for (int i = 0; i < rows_; i++)
    {
        for (int j = 0; j < cols_; j++)
        {
            Eigen::Vector4d point4(robot_gridmap_.X_(i, j), robot_gridmap_.Y_(i, j), robot_gridmap_.Z_(i, j), 1);
            point4 = T_matrix_ * point4;
            grid_map::Index terrain_idx;
            bool insubmap = terrain_sub_gridmap_.getIndex(grid_map::Position(point4.x(), point4.y()), terrain_idx);
            robot_gap_map_.X_(i, j) = point4.x();
            robot_gap_map_.Y_(i, j) = point4.y();
            robot_gap_map_.Z_(i, j) = point4.z();
            if (insubmap)
            {
                double gap = point4.z() - terrainmatrix_(terrain_idx(0), terrain_idx(1));
                robot_gap_map_.Gap_(i, j) = gap;
                min_gap = min(min_gap, gap);
            }

            min_local_z = min(min_local_z, robot_gridmap_.Z_(i, j));
        }
    }
    local_min_z_ = min_local_z;
    T_matrix_(2, 3) -= min_gap;
    T_vector_ = T_matrix_.block(0, 3, 3, 1);

    // 整体下移动最小间距，然后判断接触点
    for (int i = 0; i < rows_; i++)
    {
        for (int j = 0; j < cols_; j++)
        {
            robot_gap_map_.Z_(i, j) -= min_gap;
            robot_gap_map_.Gap_(i, j) -= min_gap;
            if (robot_gap_map_.Gap_(i, j) < d_touch_gap_)
            {
                touch_points_.push_back(Eigen::Vector3d(robot_gap_map_.X_(i, j), robot_gap_map_.Y_(i, j), robot_gap_map_.Z_(i, j)));
                touch_points_2D_.push_back(cv::Point2f(robot_gridmap_.X_(i, j), robot_gridmap_.Y_(i, j)));
            }
        }
    }
    showPose();
    showPoseInfo();

    interation_ = 1;
    ros::Rate r(10);
    while (nh_.ok())
    {
        if (!stable_)
        {
            if (interation_ > 100)
            {
                std::cout << "The interation is more than 888, exit." << std::endl;
                return;
            }

            iterateOnce();
            interation_++;

            touch_points_.clear();
            touch_points_2D_.clear();

            min_gap = 100;
            // 每次迭代后更新robot_gap_map，并判断新的接触点
            for (int i = 0; i < robot_gridmap_.X_.rows(); i++)
            {
                for (int j = 0; j < robot_gridmap_.X_.cols(); j++)
                {
                    Eigen::Vector4d point4(robot_gridmap_.X_(i, j), robot_gridmap_.Y_(i, j), robot_gridmap_.Z_(i, j), 1);
                    point4 = T_matrix_ * point4;
                    grid_map::Index terrain_idx;
                    bool insubmap = terrain_sub_gridmap_.getIndex(grid_map::Position(point4.x(), point4.y()), terrain_idx);
                    robot_gap_map_.X_(i, j) = point4.x();
                    robot_gap_map_.Y_(i, j) = point4.y();
                    robot_gap_map_.Z_(i, j) = point4.z();
                    if (insubmap)
                    {
                        double gap = point4.z() - terrainmatrix_(terrain_idx(0), terrain_idx(1));
                        robot_gap_map_.Gap_(i, j) = gap;
                        min_gap = min(min_gap, gap);

                        // bug 第二次迭代，位姿态更新后，没有发现接触点？
                        if (robot_gap_map_.Gap_(i, j) < d_touch_gap_)
                        {
                            touch_points_.push_back(Eigen::Vector3d(robot_gap_map_.X_(i, j), robot_gap_map_.Y_(i, j), robot_gap_map_.Z_(i, j)));
                            touch_points_2D_.push_back(cv::Point2f(robot_gridmap_.X_(i, j), robot_gridmap_.Y_(i, j)));
                        }
                    }
                }
            }

            if (touch_points_.size() == 0)
            {
                ROS_INFO("no touch points found! failing min_gap!");
                T_matrix_(2, 3) -= min_gap;
                T_vector_ = T_matrix_.block(0, 3, 3, 1);

                // 如果没有接触点，整体下移动最小间距，然后判断接触点
                for (int i = 0; i < rows_; i++)
                {
                    for (int j = 0; j < cols_; j++)
                    {
                        robot_gap_map_.Z_(i, j) -= min_gap;
                        robot_gap_map_.Gap_(i, j) -= min_gap;
                        if (robot_gap_map_.Gap_(i, j) < d_touch_gap_)
                        {
                            touch_points_.push_back(Eigen::Vector3d(robot_gap_map_.X_(i, j), robot_gap_map_.Y_(i, j), robot_gap_map_.Z_(i, j)));
                            touch_points_2D_.push_back(cv::Point2f(robot_gridmap_.X_(i, j), robot_gridmap_.Y_(i, j)));
                        }
                    }
                }
            }
        }
        else
        {
            ROS_INFO("Pose is stable now~");
        }

        showPose();
        showPoseInfo();
        ros::spinOnce();

        r.sleep();
    }
    return;
}

void pose_prediction::iterateOnce()
{

    if (touch_points_2D_ == local_2D_touch_points_ && !local_2D_touch_points_.empty())
    {
        touch_points_change_ = false;
    }
    else
    {
        touch_points_change_ = true;
    }
    if (touch_points_change_ && key_pushed_)
    {
        touch_poly_.clear();
        local_2D_touch_poly_.clear();
        local_2D_touch_points_ = touch_points_2D_;
        if (local_2D_touch_points_.size() <= 3)
        {
            local_2D_touch_poly_ = local_2D_touch_points_;
        }
        else
        {
            cv::convexHull(local_2D_touch_points_, local_2D_touch_poly_, false);
        }
        for (auto p : local_2D_touch_poly_)
        {
            // 用了强假设，假设车辆中仅有车轮/履带会与地面接触
            touch_poly_.push_back((T_matrix_ * Eigen::Vector4d(p.x, p.y, local_min_z_, 1)).block(0, 0, 3, 1));
        }
        calRotationLine();
        if (stable_)
        {
            return;
        }
        double d_theta = calRotationAngle();
        Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
        T1.block<3, 1>(0, 3) = -rotation_line_.origin();
        Eigen::Matrix4d R1 = Eigen::Matrix4d::Identity();
        R1.block<3, 3>(0, 0) = Eigen::Quaterniond().setFromTwoVectors(rotation_line_.direction(), Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
        Eigen::Matrix4d R2 = Eigen::Matrix4d::Identity();
        R2.block<3, 3>(0, 0) = Eigen::AngleAxisd(d_theta, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
        Eigen::Matrix4d R3 = Eigen::Matrix4d::Identity();
        R3.block<3, 3>(0, 0) = R1.block<3, 3>(0, 0).transpose();
        Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
        T2.block<3, 1>(0, 3) = rotation_line_.origin();
        Rotate_with_rotation_line_ = T2 * R3 * R2 * R1 * T1;
        T_matrix_ = Rotate_with_rotation_line_ * T_matrix_;

        T_matrix_.block<2, 1>(0, 3) = pos_;
        T_vector_ = T_matrix_.block(0, 3, 3, 1);
        R_matrix_ = T_matrix_.block(0, 0, 3, 3);
        // key_pushed_ = false;
        // debug
        std::ofstream outfile("/home/wxr/proj/traversability/obstacle_detection/ws_obdt/src/obstacle_mapping/poses/T_matrix_iterated.txt");
        if (outfile.is_open())
        {
            outfile << Rotate_with_rotation_line_ << std::endl;
            outfile.close();
            std::cout << "Matrix exported to T_matrix_iterated.txt" << std::endl;
        }
        else
        {
            std::cerr << "Error opening file for writing." << std::endl;
        }
    }
}

Eigen::Vector3d pose_prediction::fitplane(vector<Eigen::Vector3d> points)
{
    Eigen::Vector3d mean_points = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < points.size(); i++)
        mean_points += points[i];

    mean_points /= (double)points.size();

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < points.size(); i++)
    {
        Eigen::Vector3d v = points[i] - mean_points;
        cov += v * v.transpose();
    }
    cov /= (double)points.size();
    Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Matrix<double, 3, 1> D = es.pseudoEigenvalueMatrix().diagonal();
    Eigen::Matrix3d V = es.pseudoEigenvectors();
    Eigen::MatrixXd::Index evalsMax;
    D.minCoeff(&evalsMax);
    Eigen::Matrix<double, 3, 1> n = V.col(evalsMax);
    n.normalize();
    if (n(2, 0) < 0.0)
        n = -n;

    double sigma = D(evalsMax) / D.sum() * 3.0;
    if (isnan(sigma))
    {
        sigma = 1.0;
        n = Eigen::Vector3d(1.0, 0.0, 0.0);
    }

    return n;
}

void pose_prediction::calRotationLine()
{
    Eigen::Vector3d F((-Eigen::Vector3d::UnitZ()).normalized()); // 重力
    Eigen::ParametrizedLine<double, 3> F_line(T_vector_, F);
    Eigen::Vector3d z = R_matrix_.block(0, 2, 3, 1);
    Eigen::Hyperplane<double, 3> plane(z, touch_poly_.front());
    Eigen::Vector3d intersection = F_line.intersectionPoint(plane);
    Eigen::Vector3d rotatep1, rotatep2;
    if (touch_poly_.size() == 1)
    {
        if (touch_poly_.front() == intersection)
        {
            stable_ = true;
            return;
        }
        rotation_line_ = getRotationLine(intersection, touch_poly_.front(), F);
    }
    else
    {
        if (touch_poly_.size() == 2)
        {
            Eigen::Vector3d projection = getProjectePoint(touch_poly_.front(), touch_poly_.back(), intersection);
            if (projection == intersection)
            {
                stable_ = true;
                return;
            }
            // 从touch_poly_.front()到touch_poly_.back()的方向不稳定
            if (((touch_poly_.back().x() - touch_poly_.front().x()) * (intersection.y() - touch_poly_.front().y()) -
                 (intersection.x() - touch_poly_.front().x()) * (touch_poly_.back().y() - touch_poly_.front().y())) < 0)
            {
                rotatep1 = touch_poly_.front();
                rotatep2 = touch_poly_.back();
            }
            else
            {
                rotatep1 = touch_poly_.back();
                rotatep2 = touch_poly_.front();
            }
            // rotation_line_ = getRotationLine(intersection, projection, F);
        }
        else
        {
            if (ifInPoly(intersection))
            {
                stable_ = true;
                return;
            }
            else
            {
                // 遍历多边形的每一条边，找到最近的投影点，并计算旋转线
                double min_dis(FLT_MAX);
                Eigen::Vector3d projection;
                Eigen::Vector3d p1, p2;
                for (int i = 0; i < touch_poly_.size(); i++)
                {
                    p1 = touch_poly_[i];
                    if (i == touch_poly_.size() - 1)
                    {
                        p2 = touch_poly_.front();
                    }
                    else
                    {
                        p2 = touch_poly_[i + 1];
                    }
                    if (((p2.x() - p1.x()) * (intersection.y() - p1.y()) - (intersection.x() - p1.x()) * (p2.y() - p1.y())) >= 0)
                    {
                        // 这条边是稳定的，跳过
                        continue;
                    }
                    Eigen::Vector3d projection_p1p2 = getProjectePoint(p1, p2, intersection);
                    double dis = (projection_p1p2 - intersection).norm();
                    if (dis < min_dis)
                    {
                        min_dis = dis;
                        projection = projection_p1p2;
                        rotatep1 = p1;
                        rotatep2 = p2;
                    }
                }
                // rotation_line_ = getRotationLine(intersection, projection, F);
            }
        }

        // 计算方向向量
        Eigen::Vector3d direction = rotatep2 - rotatep1;

        // 创建参数化直线
        rotation_line_ = Eigen::ParametrizedLine<double, 3>(rotatep1, direction.normalized());
    }
}

Eigen::ParametrizedLine<double, 3> pose_prediction::getRotationLine(const Eigen::Vector3d &intersection, const Eigen::Vector3d &projection, const Eigen::Vector3d &F)
{
    Eigen::Vector3d projection2intersection = (intersection - projection).normalized();
    Eigen::Vector3d perpendicular = projection2intersection.cross(F);
    return Eigen::ParametrizedLine<double, 3>(projection, perpendicular);
}

Eigen::Vector3d pose_prediction::getProjectePoint(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &a)
{
    Eigen::Vector3d p1p2 = p2 - p1;
    Eigen::Vector3d p1a = a - p1;
    double dotProduct = p1a.dot(p1p2);
    double lengthSquared = p1p2.dot(p1p2);
    double t = dotProduct / lengthSquared;
    Eigen::Vector3d projection;
    // if (t < 0.0)
    // {
    //     projection = p1;
    // }
    // else if (t > 1.0)
    // {
    //     projection = p2;
    // }
    // else
    // {
    //     projection = p1 + t * p1p2;
    // }
    projection = p1 + t * p1p2;
    return projection;
}

bool pose_prediction::ifInPoly(const Eigen::Vector3d &intersection)
{
    for (int i = 0; i < touch_poly_.size(); i++)
    {
        Eigen::Vector2d p1(touch_poly_[i].x(), touch_poly_[i].y());
        Eigen::Vector2d p2;
        Eigen::Vector2d intersect(intersection.x(), intersection.y());
        if (i == touch_poly_.size() - 1)
        {
            p2.x() = touch_poly_.front().x();
            p2.y() = touch_poly_.front().y();
        }
        else
        {
            p2.x() = touch_poly_[i + 1].x();
            p2.y() = touch_poly_[i + 1].y();
        }
        Eigen::Vector2d p1p2 = p2 - p1;
        Eigen::Vector2d p1intersect = intersect - p1;

        if ((p1p2.x() * p1intersect.y() - p1intersect.x() * p1p2.y()) < 0)
        {
            return false;
        }
    }
    return true;
}

double pose_prediction::calRotationAngle()
{
    double d_theta = FLT_MAX;
    for (int i = 0; i < robot_gap_map_.X_.rows(); i++)
    {
        for (int j = 0; j < robot_gap_map_.X_.cols(); j++)
        {
            Eigen::Vector2d rotation_line_origin2cell(robot_gap_map_.X_(i, j) - rotation_line_.origin().x(), robot_gap_map_.Y_(i, j) - rotation_line_.origin().y());
            Eigen::Vector2d rotation_line_direction(rotation_line_.direction()(0), rotation_line_.direction()(1));
            if (rotation_line_origin2cell.x() * rotation_line_direction.y() - rotation_line_origin2cell.y() * rotation_line_direction.x() > 0 &&
                robot_gap_map_.Gap_(i, j) > d_touch_gap_)
            {
                Eigen::Vector2d proj_vec = rotation_line_origin2cell.dot(rotation_line_direction.normalized()) * rotation_line_direction.normalized();
                double distance = (rotation_line_origin2cell - proj_vec).norm();
                d_theta = std::min(d_theta, atan2(robot_gap_map_.Gap_(i, j), distance));
            }
        }
    }
    return d_theta;
}

void pose_prediction::showPoseInfo()
{
    visualization_msgs::Marker touch_points_marker;
    touch_points_marker.header.frame_id = "map";
    touch_points_marker.type = visualization_msgs::Marker::POINTS;
    touch_points_marker.action = visualization_msgs::Marker::ADD;
    touch_points_marker.scale.x = 0.2;
    touch_points_marker.scale.y = 0.2;
    touch_points_marker.scale.z = 0.2;
    touch_points_marker.ns = "touch_points";
    ROS_INFO("touch_points size: %d", touch_points_.size());
    for (size_t i = 0; i < touch_points_.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = touch_points_[i].x();
        p.y = touch_points_[i].y();
        p.z = touch_points_[i].z();
        touch_points_marker.points.push_back(p);
    }
    touch_points_marker.color = std_msgs::ColorRGBA();
    touch_points_marker.color.r = 1.0;
    touch_points_marker.color.g = 0.0;
    touch_points_marker.color.b = 0.0;
    touch_points_marker.color.a = 1.0;

    touch_points_marker.pose.orientation.x = 0;
    touch_points_marker.pose.orientation.y = 0;
    touch_points_marker.pose.orientation.z = 0;
    touch_points_marker.pose.orientation.w = 1;

    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "line_visualization";
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;

    line_marker.color = std_msgs::ColorRGBA();
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.0;
    line_marker.color.b = 1.0;
    line_marker.color.a = 1.0;

    line_marker.scale.x = 0.1;

    // 计算起点和终点
    Eigen::Vector3d point_on_line = rotation_line_.origin();
    Eigen::Vector3d direction = rotation_line_.direction();
    Eigen::Vector3d endpoint = point_on_line + direction * 2.0;

    // 添加起点和终点
    geometry_msgs::Point start;
    start.x = point_on_line.x();
    start.y = point_on_line.y();
    start.z = point_on_line.z();
    line_marker.points.push_back(start);

    geometry_msgs::Point end;
    end.x = endpoint.x();
    end.y = endpoint.y();
    end.z = endpoint.z();
    line_marker.points.push_back(end);

    marker_pub_.publish(touch_points_marker);
    return;
}

void pose_prediction::showPose()
{
    tf::Transform transform;

    // 提取旋转部分
    Eigen::Matrix3d rotation = T_matrix_.block<3, 3>(0, 0);
    Eigen::Quaterniond q(rotation);

    // 提取平移部分
    Eigen::Vector3d translation = T_matrix_.block<3, 1>(0, 3);

    // 设置变换
    transform.setOrigin(tf::Vector3(translation.x(), translation.y(), translation.z()));
    transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

    // 发布变换
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "robot"));
    return;
}

void pose_prediction::KeyCallback(const std_msgs::Bool::ConstPtr key)
{
    if (key->data == true)
    {
        key_pushed_ = true;
    }
}

void pose_prediction::pubishmap(const ros::TimerEvent &event)
{
    grid_map_msgs::GridMap grid_map_msg, grid_map_msg_sub, robot_gridmap_msg;

    grid_map_msg.info.header.stamp = ros::Time::now();
    grid_map::Index idx(2, 10);
    grid_map::GridMapRosConverter::toMessage(terrain_gridmap_, grid_map_msg);
    terrain_map_pub_.publish(grid_map_msg);

    grid_map_msg_sub.info.header.stamp = ros::Time::now();
    grid_map::GridMapRosConverter::toMessage(terrain_sub_gridmap_, grid_map_msg_sub);
    terrain_submap_pub_.publish(grid_map_msg_sub);

    grid_map::GridMap robot_gridmap;
    grid_map::Length mapLength(3.2, 5.4);
    robot_gridmap.setFrameId("map");
    robot_gridmap.setGeometry(mapLength, 0.2, grid_map::Position(pos_.x(), pos_.y()));
    robot_gridmap.add("elevation", robot_gap_map_.Z_.cast<float>());
    robot_gridmap_msg.info.header.stamp = ros::Time::now();
    grid_map::GridMapRosConverter::toMessage(robot_gridmap, grid_map_msg_sub);
    robot_gap_map_pub_.publish(grid_map_msg_sub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_prediction");
    ros::NodeHandle nh;

    pose_prediction pose_predictor(nh);
    pose_predictor.predict();
}

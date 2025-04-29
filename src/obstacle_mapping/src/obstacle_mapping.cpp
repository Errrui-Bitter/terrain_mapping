/**
 * * * @description:
 * * * @filename: obstacle_mapping.cpp
 * * * @author: wangxurui
 * * * @date: 2025-03-14 21:05:18
 **/

#include "obstacle_mapping.h"

using namespace std;

obstacle_mapping::obstacle_mapping(ros::NodeHandle &nh) : nh_(nh)
{
    init();
}

obstacle_mapping::~obstacle_mapping()
{
}

void obstacle_mapping::init()
{
    /****************************
    init params
    ****************************/
    nh_.param<int>("mapping_mode", mapping_mode_, 1);
    nh_.param<int>("threadCount", threadCount_, 1);
    nh_.param<string>("lidar_topic", lidar_topic_, "/velodyne_points");
    nh_.param<string>("frame_id/map", map_frame_, "map");
    nh_.param<string>("frame_id/robot", robot_frame_, "base_link");
    nh_.param<string>("frame_id/lidar", lidar_frame_, "velodyne");
    nh_.param<double>("local_map/resolution", local_map_resolution_, 0.2);
    nh_.param<double>("local_map/mapLengthX", local_map_length_x_, 40.0);
    nh_.param<double>("local_map/mapLengthY", local_map_length_y_, 60.0);
    nh_.param<double>("global_map/resolution", global_map_resolution_, 0.2);
    nh_.param<double>("global_map/mapLengthX", global_map_length_x_, 1000.0);
    nh_.param<double>("global_map/mapLengthY", global_map_length_y_, 1000.0);
    nh_.param<double>("global_map/submapLengthX", global_submap_length_x_, 100.0);
    nh_.param<double>("global_map/submapLengthY", global_submap_length_y_, 100.0);
    nh_.param<double>("global_map/sensorRangeLimit", sensorRangeLimit_, 80.0);
    nh_.param<double>("global_map/predictionKernalSize", predictionKernalSize_, 1.0);
    nh_.param<double>("minZ", minZ_, -3.0);
    nh_.param<double>("maxZ", maxZ_, 4.0);
    nh_.param<double>("lidar_Z", lidar_z_, 0);
    nh_.param<double>("heightdiff_threshold", heightdiffthreshold_, 0.6);
    nh_.param<double>("cntratiothreshold", cntratiothreshold_, 0.5);
    nh_.param<double>("normal_estimationRadius", normal_estimationRadius_, 0.5);

    /****************************
    init gridmap
    ****************************/

    initgridmap(local_map_, robot_frame_, local_map_resolution_, local_map_length_x_, local_map_length_y_);
    initgridmap(fused_local_map_, robot_frame_, local_map_resolution_, local_map_length_x_, local_map_length_y_);
    initgridmap(global_map_, map_frame_, global_map_resolution_, global_map_length_x_, global_map_length_y_);

    /****************************
    init ros
    ****************************/
    lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(lidar_topic_, 1, boost::bind(&obstacle_mapping::Mapping, this, _1));
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/liorf/mapping/odometry", 1, boost::bind(&obstacle_mapping::StoreOdom, this, _1));

    local_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/localmap", 1);
    local_fused_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/localfusedmap", 1);
    global_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/globalmap", 1);
}

void obstacle_mapping::initgridmap(grid_map::GridMap &map, string frame, double height_map_resolution, double mapLength_x, double mapLength_y)
{
    grid_map::Length mapLength(mapLength_x, mapLength_y);
    map.setFrameId(frame);
    map.setGeometry(mapLength, height_map_resolution, grid_map::Position(0.0, 0.0));
    map.add("elevation");
    map.add("variance");
    map.add("min_elevation");
    map.add("max_elevation");
    map.add("normal_z");
    map.add("roughness");
    map.add("step");
    map.add("traversability");
    map.add("n_points", 0);
    map.add("obstacle", 0);
    map.add("obstacle_cnt", 0);
}

void obstacle_mapping::StoreOdom(const nav_msgs::OdometryConstPtr &odom)
{
    odom_deque_.push_back(*odom);
    if (odom_deque_.size() > 50)
        odom_deque_.pop_front();
}

void obstacle_mapping::Mapping(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    map_time_ = msg->header.stamp;
    auto inputcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *inputcloud);
    /****************************
    单帧地图模式
    ****************************/
    if (mapping_mode_ == 1)
    {
        ROS_INFO("Local Mapping...");
        auto localcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        localcloud->clear();
        localcloud->reserve(inputcloud->size());
        for (auto &point : inputcloud->points)
        {
            point.z += lidar_z_;
            if (point.z >= minZ_ && point.z <= maxZ_)
            {
                localcloud->points.push_back(point);
            }
        }
        LocalMapping(*localcloud);
        obstacledetection(local_map_);
        publocalmap();
        local_map_.clearAll();
        int rows = local_map_length_x_ / local_map_resolution_;
        int cols = local_map_length_y_ / local_map_resolution_;
        Eigen::MatrixXf zeroMatrix = Eigen::MatrixXf::Zero(rows, cols);
        local_map_.add("n_points", zeroMatrix);
        local_map_.add("obstacle", zeroMatrix);
        local_map_.add("obstacle_cnt", zeroMatrix);
        return;
    }

    /****************************
    多帧融合模式
    ****************************/
    else if (mapping_mode_ == 2)
    {
        ROS_INFO("Local Fusion Mapping...");
        auto localcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        localcloud->clear();
        localcloud->reserve(inputcloud->size());
        for (auto &point : inputcloud->points)
        {
            point.z += lidar_z_;
            if (point.z >= minZ_ && point.z <= maxZ_)
            {
                localcloud->points.push_back(point);
            }
        }
        LocalMapping(*localcloud);
        FuseMap();
        pubfusedlocalmap();
        local_map_.clearAll();
    }

    /****************************
    全局地图模式
    ****************************/
    else if (mapping_mode_ == 3)
    {
        ROS_INFO("Global Mapping...");
        /****************************
        将点云坐标转到map（全局）坐标系
        ****************************/
        lidar_frame_ = msg->header.frame_id;
        tf::StampedTransform lidarmaptransform;
        try
        {
            // listener_.waitForTransform(map_frame_, lidar_frame_, ros::Time(0), ros::Duration(0.1));
            listener_.lookupTransform(map_frame_, lidar_frame_, ros::Time(0), lidarmaptransform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
        Eigen::Affine3d lmtransformMatrix = Eigen::Affine3d::Identity();
        poseTFToEigen(lidarmaptransform, lmtransformMatrix);
        robot_pose_ = lmtransformMatrix.translation();
        ROS_INFO("robot position: %.2f, %.2f, %.2f", robot_pose_.x(), robot_pose_.y(), robot_pose_.z());

        auto globalcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        globalcloud->clear();
        globalcloud->reserve(inputcloud->size());
        pcl::transformPointCloud(*inputcloud, *globalcloud, lmtransformMatrix);
        auto globalcloudprocessed = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        globalcloudprocessed->clear();
        globalcloudprocessed->reserve(globalcloud->size());
        for (auto &point : globalcloud->points)
        {
            point.z += lidar_z_;
            if (point.z >= minZ_ && point.z <= maxZ_)
            {
                globalcloudprocessed->points.push_back(point);
            }
        }

        /****************************
        全局建图
        ****************************/
        GlobalMapping(*globalcloud);
        // 子图中心点
        center_robot_ = robot_pose_.head<2>();
        if (!global_map_.getIndex(center_robot_, center_robot_index_))
        {
            ROS_ERROR("robot is out of global map !");
            return;
        }
        // 提取子图
        bool submap_sucess;
        grid_map::Length submap_length(global_submap_length_x_, global_submap_length_y_);
        global_submap_ = global_map_.getSubmap(center_robot_, submap_length, submap_sucess);
        if (!submap_sucess)
        {
            ROS_WARN("cound not get the global_submap!!!");
            return;
        }
        double time1 = ros::Time::now().toSec();
        DenseMapping(global_submap_);
        double time2 = ros::Time::now().toSec();
        NormalMapping(global_submap_);
        double time3 = ros::Time::now().toSec();

        // MergetoGlobalMap();

        ROS_INFO("DenseMapping takes time : %f s", time2 - time1);
        ROS_INFO("NormalMapping takes time : %f s", time3 - time2);
        pubglobalmap();
    }
    else
    {
        ROS_ERROR("Mapping mode %d is doesnot exist!!", mapping_mode_);
    }
    return;
}

void obstacle_mapping::LocalMapping(const pcl::PointCloud<pcl::PointXYZ> &localcloud)
{
    auto &heightMatrix = local_map_["elevation"];
    auto &varianceMatrix = local_map_["variance"];
    auto &minHeightMatrix = local_map_["min_elevation"];
    auto &maxHeightMatrix = local_map_["max_elevation"];
    auto &npointsMatrix = local_map_["n_points"];
    grid_map::Index PointIndex;
    grid_map::Position PointPosition;

    for (const auto &Point : localcloud)
    {
        PointPosition << Point.x, Point.y;
        if (!local_map_.getIndex(PointPosition, PointIndex))
        {
            // ROS_INFO("outline npoint: %.2f", Point.x);
            continue;
        }
        auto &height = heightMatrix(PointIndex(0), PointIndex(1));
        auto &variance = varianceMatrix(PointIndex(0), PointIndex(1));
        auto &npoints = npointsMatrix(PointIndex(0), PointIndex(1));
        auto &minHeight = minHeightMatrix(PointIndex(0), PointIndex(1));
        auto &maxHeight = maxHeightMatrix(PointIndex(0), PointIndex(1));
        if (!npoints)
        {
            height = Point.z;
            variance = 0.0f;
            minHeight = Point.z;
            maxHeight = Point.z;
            npoints += 1;
        }
        else
        {
            // height = (npoints / (npoints + 1)) * height + 1 / (npoints + 1) * Point.z;
            npoints += 1;
            updateHeightStats(height, variance, npoints, Point.z);
            minHeight = std::min(minHeight, Point.z);
            maxHeight = std::max(maxHeight, Point.z);
        }
    }
}

void obstacle_mapping::GlobalMapping(const pcl::PointCloud<pcl::PointXYZ> &globalcloud)
{
    auto &heightMatrix = global_map_["elevation"];
    auto &npointsMatrix = global_map_["n_points"];
    auto &varianceMatrix = global_map_["variance"];
    auto &minHeightMatrix = global_map_["min_elevation"];
    auto &maxHeightMatrix = global_map_["max_elevation"];
    grid_map::Index PointIndex;
    grid_map::Position PointPosition;

    for (const auto &Point : globalcloud)
    {
        PointPosition << Point.x, Point.y;
        if (!global_map_.getIndex(PointPosition, PointIndex))
            continue;
        auto &height = heightMatrix(PointIndex(0), PointIndex(1));
        auto &npoints = npointsMatrix(PointIndex(0), PointIndex(1));
        auto &variance = varianceMatrix(PointIndex(0), PointIndex(1));
        auto &minHeight = minHeightMatrix(PointIndex(0), PointIndex(1));
        auto &maxHeight = maxHeightMatrix(PointIndex(0), PointIndex(1));
        if (!npoints)
        {
            height = Point.z;
            minHeight = Point.z;
            maxHeight = Point.z;
        }
        else
        {
            updateHeightStats(height, variance, npoints, Point.z);
            minHeight = std::min(minHeight, Point.z);
            maxHeight = std::max(maxHeight, Point.z);
        }
        npoints += 1;
    }
}

void obstacle_mapping::DenseMapping(grid_map::GridMap &map)
{
    int map_num_x = map.getSize().x();
    int map_num_y = map.getSize().y();
    double map_resolution = map.getResolution();
    double map_length_x = map.getLength().x();
    double map_length_y = map.getLength().y();
    int densenum = 0;

    auto &heightMatrix = map["elevation"];
    auto &minHeightMatrix = map["min_elevation"];
    auto &varianceMatrix = map["variance"];
    auto &maxHeightMatrix = map["max_elevation"];
    auto &npointsMatrix = map["n_points"];
    grid_map::GridMap::Matrix heightMatrix_BGK = heightMatrix;

    for (grid_map::CircleIterator it(map, center_robot_, sensorRangeLimit_); !it.isPastEnd(); ++it)
    {
        grid_map::Index index = *it;

        // skip observed point
        if (npointsMatrix(index(0), index(1)) != 0)
        {
            continue;
        }
        Eigen::Vector2d testpoint;
        bool inmap = map.getPosition(index, testpoint);
        // skip grids too close
        double pointDistance = (testpoint - robot_pose_.head<2>()).norm();
        if (pointDistance < 1.0)
        {
            continue;
        }
        // Training data
        vector<float> xTrainVec;     // training data x and y coordinates
        vector<float> yTrainVecElev; // training data elevation
        vector<float> yTrainVecOccu; // training data occupancy
        grid_map::Position center(testpoint(0), testpoint(1));
        for (grid_map::CircleIterator cit(map, center, predictionKernalSize_); !cit.isPastEnd(); ++cit)
        {
            grid_map::Index cindex = *cit;
            // save only observed grid in this scan
            if (npointsMatrix(cindex(0), cindex(1)) == 0)
            {
                continue;
            }
            Eigen::Vector3d trainpoint;
            if (map.getPosition3("elevation", cindex, trainpoint))
            {
                xTrainVec.push_back(trainpoint(0));
                xTrainVec.push_back(trainpoint(1));
                yTrainVecElev.push_back(trainpoint(2));
            }
        }
        // no training data available, continue
        // if (xTrainVec.size() == 0)
        if (xTrainVec.size() < 3)
            continue;
        // convert from vector to eigen
        Eigen::MatrixXf xTrain = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTrainVec.data(), xTrainVec.size() / 2, 2);
        Eigen::MatrixXf yTrainElev = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVecElev.data(), yTrainVecElev.size(), 1);
        // Test data (current grid)
        vector<float> xTestVec;
        xTestVec.push_back(testpoint(0));
        xTestVec.push_back(testpoint(1));
        Eigen::MatrixXf xTest = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTestVec.data(), xTestVec.size() / 2, 2);
        // Predict
        Eigen::MatrixXf Ks;           // covariance matrix
        covSparse(xTest, xTrain, Ks); // sparse kernel

        Eigen::MatrixXf ybarElev = (Ks * yTrainElev).array();
        Eigen::MatrixXf kbar = Ks.rowwise().sum().array();

        // Update Elevation with Prediction
        if (std::isnan(ybarElev(0, 0)) || std::isnan(kbar(0, 0)))
            continue;

        if (kbar(0, 0) == 0)
            continue;

        float elevation = ybarElev(0, 0) / kbar(0, 0);
        npointsMatrix(index(0), index(1)) += 1;
        heightMatrix_BGK(index(0), index(1)) = elevation;
        // debug
        densenum++;
    }
    map.add("elevation_BGK", heightMatrix_BGK);
    if (densenum)
        ROS_INFO("Dense map grids : %d", densenum);
}

void StepMapping(grid_map::GridMap &map)
{
}

// __global__ void computeNormalsKernel(const float *elevationData, const int width, const int height,
//                                      const double estimationRadius, float *normalData, int *validCounts)
// {
//     int idx = blockIdx.x * blockDim.x + threadIdx.x;
//     if (idx >= width * height)
//         return;

//     int x = idx / height;
//     int y = idx % height;

//     // Check if the cell is valid
//     if (elevationData[idx] == -1.0f)
//         return; // Assuming -1.0f indicates invalid

//     // Initialize variables for normal calculation
//     int nPoints = 0;
//     Eigen::Vector3d sum = Eigen::Vector3d::Zero();
//     Eigen::Matrix3d sumSquared = Eigen::Matrix3d::Zero();

//     // Gather surrounding data within the estimation radius
//     for (int dx = -estimationRadius; dx <= estimationRadius; ++dx)
//     {
//         for (int dy = -estimationRadius; dy <= estimationRadius; ++dy)
//         {
//             int neighborX = x + dx;
//             int neighborY = y + dy;

//             if (neighborX >= 0 && neighborX < width && neighborY >= 0 && neighborY < height)
//             {
//                 float neighborElevation = elevationData[neighborX * height + neighborY];
//                 if (neighborElevation != -1.0f)
//                 {
//                     nPoints++;
//                     Eigen::Vector3d point(neighborX, neighborY, neighborElevation);
//                     sum += point;
//                     sumSquared.noalias() += point * point.transpose();
//                 }
//             }
//         }
//     }

//     // Compute normal vector
//     if (nPoints >= 3)
//     {
//         Eigen::Vector3d mean = sum / nPoints;
//         Eigen::Matrix3d covarianceMatrix = sumSquared / nPoints - mean * mean.transpose();
//         Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covarianceMatrix);
//         if (solver.eigenvalues()(1) > 1e-8)
//         {
//             Eigen::Vector3d normalVector = solver.eigenvectors().col(0);
//             if (normalVector.dot(Vector3::UnitZ()) < 0.0)
//             {
//                 normalVector = -normalVector;
//             }
//             normalData[idx] = normalVector.z();
//         }
//     }
//     validCounts[idx] = nPoints;
// }

// void obstacle_mapping::NormalMapping_CUDA(grid_map::GridMap &map)
// {
//     const double start = ros::Time::now().toSec();
//     grid_map::Size gridMapSize = map.getSize();
//     double map_res = map.getResolution();
//     auto &heightMatrix = map["elevation"];
//     auto &normalzMatrix = map["normal_z"];

//     // Prepare CUDA data
//     float *d_elevationData;
//     float *d_normalData;
//     int *d_validCounts;
//     size_t gridSize = gridMapSize(0) * gridMapSize(1);

//     // Allocate memory on the device
//     cudaMalloc(&d_elevationData, gridSize * sizeof(float));
//     cudaMalloc(&d_normalData, gridSize * sizeof(float)); // for normal vectors
//     cudaMalloc(&d_validCounts, gridSize * sizeof(int));

//     // Copy elevation data to device
//     float *elevationData = new float[gridSize];
//     for (int i = 0; i < gridMapSize(0); ++i)
//     {
//         for (int j = 0; j < gridMapSize(1); ++j)
//         {
//             elevationData[i * gridMapSize(1) + j] = map.at("elevation", grid_map::Index(i, j));
//         }
//     }
//     cudaMemcpy(d_elevationData, elevationData, gridSize * sizeof(float), cudaMemcpyHostToDevice);

//     // Launch CUDA kernel
//     int threadsPerBlock = 256;
//     int blocks = (gridSize + threadsPerBlock - 1) / threadsPerBlock;
//     computeNormalsKernel<<<blocks, threadsPerBlock>>>(d_elevationData, gridMapSize(0), gridMapSize(1),
//                                                       ceil(normal_estimationRadius_ / map_res), d_normalData, d_validCounts);

//     // Copy results back to host
//     Eigen::Vector3d *normalResults = new Eigen::Vector3d[gridSize];
//     cudaMemcpy(normalResults, d_normalData, gridSize * sizeof(float), cudaMemcpyDeviceToHost);

//     // Store results in the map
//     for (int i = 0; i < gridMapSize(0); ++i)
//     {
//         for (int j = 0; j < gridMapSize(1); ++j)
//         {
//             normalzMatrix(i, j) = normalResults[i * gridMapSize(1) + j];
//         }
//     }

//     // Free device memory
//     cudaFree(d_elevationData);
//     cudaFree(d_normalData);
//     cudaFree(d_validCounts);
//     delete[] elevationData;
//     delete[] normalResults;

//     const double end = ros::Time::now().toSec();
//     ROS_DEBUG_THROTTLE(2.0, "NORMAL COMPUTATION TIME = %f", (end - start));
// }

void obstacle_mapping::NormalMapping(grid_map::GridMap &map)
{
    const double start = ros::Time::now().toSec();
    grid_map::Size gridMapSize = map.getSize();
    int robot_range = center_robot_index_(0) * gridMapSize(1) + center_robot_index_(1);
    // Set number of thread to use for parallel programming.
    std::unique_ptr<tbb::task_scheduler_init> TBBInitPtr;
    if (threadCount_ != -1)
    {
        TBBInitPtr.reset(new tbb::task_scheduler_init(threadCount_));
    }
    // Parallelized iteration through the map.
    tbb::parallel_for(0, gridMapSize(0) * gridMapSize(1), [&](int range)
                      {
    // Recover Cell index from range iterator.
    const grid_map::Index index(range / gridMapSize(1), range % gridMapSize(1));
    // grid_map::Position pos;
    // map.getPosition(index, pos);
    // bool insubmap = global_submap_.isInside(pos);
    bool insubmap = fabs(robot_range - range) < global_submap_length_y_ / global_map_resolution_ * global_submap_length_x_ / global_map_resolution_ / 2;
    if (map.isValid(index, "elevation") && insubmap) 
    {
        areaSingleNormalComputation(map, index);
    } });

    const double end = ros::Time::now().toSec();
    ROS_DEBUG_THROTTLE(2.0, "NORMAL COMPUTATION TIME = %f", (end - start));
}

void obstacle_mapping::areaSingleNormalComputation(grid_map::GridMap &map, const grid_map::Index &index)
{
    // Requested position (center) of circle in map.
    grid_map::Position center;
    map.getPosition(index, center);

    // Prepare data computation. Check if area is bigger than cell.
    const double minAllowedEstimationRadius = 0.5 * map.getResolution();
    if (normal_estimationRadius_ <= minAllowedEstimationRadius)
    {
        ROS_WARN("Estimation radius is smaller than allowed by the map resolution (%f < %f)", normal_estimationRadius_, minAllowedEstimationRadius);
    }

    // Gather surrounding data.
    size_t nPoints = 0;
    grid_map::Position3 sum = grid_map::Position3::Zero();
    Eigen::Matrix3d sumSquared = Eigen::Matrix3d::Zero();
    for (grid_map::CircleIterator circleIterator(map, center, normal_estimationRadius_); !circleIterator.isPastEnd(); ++circleIterator)
    {
        grid_map::Position3 point;
        if (!map.getPosition3("elevation", *circleIterator, point))
        {
            continue;
        }
        nPoints++;
        sum += point;
        sumSquared.noalias() += point * point.transpose();
    }

    Eigen::Vector3d unitaryNormalVector = Eigen::Vector3d::Zero();
    if (nPoints < 3)
    {
        ROS_DEBUG("Not enough points to establish normal direction (nPoints = %i)", static_cast<int>(nPoints));
        unitaryNormalVector = Eigen::Vector3d::UnitZ();
    }
    else
    {
        const grid_map::Position3 mean = sum / nPoints;
        const Eigen::Matrix3d covarianceMatrix = sumSquared / nPoints - mean * mean.transpose();

        // Compute Eigenvectors.
        // Eigenvalues are ordered small to large.
        // Worst case bound for zero eigenvalue from : https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
        solver.computeDirect(covarianceMatrix, Eigen::DecompositionOptions::ComputeEigenvectors);
        if (solver.eigenvalues()(1) > 1e-8)
        {
            unitaryNormalVector = solver.eigenvectors().col(0);
        }
        else
        { // If second eigenvalue is zero, the normal is not defined.
            ROS_DEBUG("Covariance matrix needed for eigen decomposition is degenerated.");
            ROS_DEBUG("Expected cause: data is on a straight line (nPoints = %i)", static_cast<int>(nPoints));
            unitaryNormalVector = Eigen::Vector3d::UnitZ();
        }
    }

    // Check direction of the normal vector and flip the sign towards the user defined direction.
    if (unitaryNormalVector.dot(Eigen::Vector3d::UnitZ()) < 0.0)
    {
        unitaryNormalVector = -unitaryNormalVector;
    }
    map.at("normal_z", index) = unitaryNormalVector.z();
}

void obstacle_mapping::obstacledetection(grid_map::GridMap &map)
{
    auto &minHeightMatrix = map["min_elevation"];
    auto &maxHeightMatrix = map["max_elevation"];
    auto &npointsMatrix = map["n_points"];
    auto &obstacleMatrix = map["obstacle"];
    grid_map::Index index;

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
    {
        index = *it;
        auto &npoints = npointsMatrix(index(0), index(1));
        auto &minHeight = minHeightMatrix(index(0), index(1));
        auto &maxHeight = maxHeightMatrix(index(0), index(1));
        auto &obstacle = obstacleMatrix(index(0), index(1));
        if (!npoints)
        {
            continue;
        }
        else
        {
            if (maxHeight - minHeight >= heightdiffthreshold_)
            {
                obstacle = 1;
            }
        }
    }
}

void obstacle_mapping::obstacledetection(const pcl::PointCloud<pcl::PointXYZ> &localcloud, grid_map::GridMap &map)
{
    auto &minHeightMatrix = map["min_elevation"];
    auto &npointsMatrix = map["n_points"];
    auto &obstaclecntMatrix = map["obstacle_cnt"];
    auto &obstacleMatrix = map["obstacle"];

    for (const auto &point : localcloud.points)
    {
        grid_map::Index index;
        grid_map::Position pos2d(point.x, point.y);
        if (map.getIndex(pos2d, index))
        {
            auto &minHeight = minHeightMatrix(index(0), index(1));
            auto &obstacle_cnt = obstaclecntMatrix(index(0), index(1));
            if (point.z - minHeight > heightdiffthreshold_)
            {
                obstacle_cnt += 1;
            }
        }
    }

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
    {
        grid_map::Index index = *it;
        auto &npoints = npointsMatrix(index(0), index(1));
        auto &obstacle_cnt = obstaclecntMatrix(index(0), index(1));
        auto &obstacle = obstacleMatrix(index(0), index(1));
        if (obstacle_cnt)
        {
            double cntratio = obstacle_cnt / npoints;
            if (cntratio >= cntratiothreshold_ ||
                obstacle_cnt > cntthreshold_)
            {
                obstacle = 1;
            }
        }
    }
}
void obstacle_mapping::FuseMap()
{
    int matchodom_index = TimestampMatch(map_time_.toSec(), odom_deque_);
    cur_odom_ = odom_deque_[matchodom_index];
    if (first_map_init_)
    {
        first_map_init_ = false;
        fused_local_map_ = local_map_;
        last_odom_ = cur_odom_;
        return;
    }

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    pcl::PointCloud<pcl::PointXYZ> historymap4transform, map4transform;
    // 计算当前里程计方向角yaw
    double odom_x = cur_odom_.pose.pose.orientation.x;
    double odom_y = cur_odom_.pose.pose.orientation.y;
    double odom_z = cur_odom_.pose.pose.orientation.z;
    double odom_w = cur_odom_.pose.pose.orientation.w;
    double siny_cosp = 2 * (odom_w * odom_z + odom_x * odom_y);
    double cosy_cosp = 1 - 2 * (odom_y * odom_y + odom_z * odom_z);
    double yaw_odom = atan2(siny_cosp, cosy_cosp);
    // 计算历史地图里程计方向角yaw
    double map_x = last_odom_.pose.pose.orientation.x;
    double map_y = last_odom_.pose.pose.orientation.y;
    double map_z = last_odom_.pose.pose.orientation.z;
    double map_w = last_odom_.pose.pose.orientation.w;
    double siny_cosp_map = 2 * (map_w * map_z + map_x * map_y);
    double cosy_cosp_map = 1 - 2 * (map_y * map_y + map_z * map_z);
    double yaw_map = atan2(siny_cosp_map, cosy_cosp_map);

    transform.translation() << -last_odom_.pose.pose.position.x + cur_odom_.pose.pose.position.x,
        -last_odom_.pose.pose.position.y + cur_odom_.pose.pose.position.y,
        -last_odom_.pose.pose.position.z + cur_odom_.pose.pose.position.z;
    transform.rotate(Eigen::AngleAxisf(yaw_odom - yaw_map, Eigen::Vector3f::UnitZ()));

    grid_map::Index index_his, index_cur;

    pcl::transformPointCloud(historymap4transform, map4transform, transform); // 通过转换矩阵transform将historymap4transform转换后存到map4transform

    for (grid_map::GridMapIterator it(fused_local_map_); !it.isPastEnd(); ++it)
    {
        index_his = *it;
        double height = fused_local_map_.at("elevation", index_his);
        double npoints = fused_local_map_.at("n_points", index_his);
        double obstacle = fused_local_map_.at("obstacle", index_his);
        double probability = fused_local_map_.at("probability", index_his);

        if (!std::isnan(height))
        {
            Eigen::Vector2d position;
            fused_local_map_.getPosition(index_his, position);

            Eigen::Vector3f point(position.x(), position.y(), 0.0);
            Eigen::Vector3f transformed_point;
            pcl::transformPoint(point, transformed_point, transform);

            Eigen::Vector2d transformed_position(transformed_point(0), transformed_point(1));
            bool inside_map = local_map_.getIndex(transformed_position, index_cur);
            if (!inside_map)
            {
                continue;
            }
            double height_raw = local_map_.at("elevation", index_cur);
            double npoints_raw = local_map_.at("n_points", index_cur);
            // 高度更新
            local_map_.at("elevation", index_cur) = (npoints * height + npoints_raw * height_raw) / (npoints + npoints_raw);
            // 点数量更新
            local_map_.at("n_points", index_cur) += npoints;
            // 障碍物更新
            if (local_map_.at("obstacle", index_cur) == 1.0)
            {
                local_map_.at("probability", index_cur) = BayesUpdator(probability, 0.7);
            }
            else
            {
                local_map_.at("probability", index_cur) = BayesUpdator(probability, 0.2);
            }
        }
    }
    fused_local_map_ = local_map_;
    return;
}

double obstacle_mapping::BayesUpdator(double value_update, double value_observe)
{
    double tem_value = 0;
    if (value_update <= 0)
    {
        tem_value = value_observe;
        return tem_value;
    }
    double tem_odds = (value_update * value_observe) / (1 - value_observe - value_update + value_observe * value_update);
    tem_value = tem_odds / (1 + tem_odds);
    if (tem_value > 0.9)
        tem_value = 0.9;
    if (tem_value < 0.2)
        tem_value = 0.2;
    return tem_value;
}

int obstacle_mapping::TimestampMatch(double fixtimestamp, std::deque<nav_msgs::Odometry> target_deque)
{
    int index = 0;
    double min_diff = 100;
    for (int i = 0; i < target_deque.size(); i++)
    {
        double timediff = fabs(target_deque[i].header.stamp.toSec() - fixtimestamp);
        if (timediff < min_diff)
        {
            min_diff = timediff;
            index = i;
        }
    }
    return index;
}

void obstacle_mapping::updateHeightStats(float &height, float &variance, float n, float new_height)
{

    const float delta = (new_height - height);
    const float delta_n = delta / n;

    // Update mean
    height += delta_n;

    // Update variance using the more stable algorithm
    // M2 = M2 + delta * (new_value - updated_mean)
    // where M2 is the sum of squared differences from the mean
    variance += delta * (new_height - height);

    // Convert M2 to variance
    if (n > 1)
    {
        variance = variance / (n - 1); // Use (n-1) for sample variance
    }
}

void obstacle_mapping::dist(const Eigen::MatrixXf &xStar, const Eigen::MatrixXf &xTrain, Eigen::MatrixXf &d) const
{
    d = Eigen::MatrixXf::Zero(xStar.rows(), xTrain.rows());
    for (int i = 0; i < xStar.rows(); ++i)
    {
        d.row(i) = (xTrain.rowwise() - xStar.row(i)).rowwise().norm();
    }
}

void obstacle_mapping::covSparse(const Eigen::MatrixXf &xStar, const Eigen::MatrixXf &xTrain, Eigen::MatrixXf &Kxz) const
{
    dist(xStar / (predictionKernalSize_ + 0.1), xTrain / (predictionKernalSize_ + 0.1), Kxz);
    Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) * (1.0f - Kxz.array()) / 3.0f) +
           (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f))
              .matrix() *
          1.0f;
    // Clean up for values with distance outside length scale, possible because Kxz <= 0 when dist >= predictionKernalSize
    for (int i = 0; i < Kxz.rows(); ++i)
        for (int j = 0; j < Kxz.cols(); ++j)
            if (Kxz(i, j) < 0)
                Kxz(i, j) = 0;
}

void obstacle_mapping::publocalmap()
{
    grid_map_msgs::GridMap msg;
    msg.info.header.stamp = map_time_;
    grid_map::GridMapRosConverter::toMessage(local_map_, msg);
    local_map_pub_.publish(msg);
}

void obstacle_mapping::pubfusedlocalmap()
{
    grid_map_msgs::GridMap msg;
    msg.info.header.stamp = map_time_;
    grid_map::GridMapRosConverter::toMessage(fused_local_map_, msg);
    local_fused_map_pub_.publish(msg);
}

void obstacle_mapping::pubglobalmap()
{
    grid_map_msgs::GridMap msg;
    msg.info.header.stamp = map_time_;
    grid_map::GridMapRosConverter::toMessage(global_submap_, msg);
    global_map_pub_.publish(msg);
}
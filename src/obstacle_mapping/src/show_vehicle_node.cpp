/**
 * * * @description:
 * * * @filename: show_vehicle_node.cpp
 * * * @author: wangxurui
 * * * @date: 2025-05-13 17:40:12
 **/

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class PointSubscriber
{
public:
    PointSubscriber()
    {
        nh_ = ros::NodeHandle("~");

        point_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &PointSubscriber::pointCallback, this);

        test_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
        cur_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/cur_marker", 10);

        nh_.param<double>("/robot_scale", scale_, 0.8);
        nh_.param<double>("/pred_alpha", pred_alpha_, 0.8);
        nh_.param<double>("/vehicle_height", vehicle_height_, 0.8);
        geometry_msgs::PoseStamped goal = geometry_msgs::PoseStamped();
        test_pub_.publish(goal);

        visualization_msgs::Marker marker_6t_;
        marker_6t_.header.frame_id = "map";
        marker_6t_.header.stamp = ros::Time::now();
        marker_6t_.ns = "cur";
        marker_6t_.id = 0;
        marker_6t_.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker_6t_.mesh_resource = "file:///home/wxr/ivrc/6t/catkin_marker/src/base_link.dae";
        marker_6t_.action = visualization_msgs::Marker::ADD;
        marker_6t_.pose.position.x = 0;
        marker_6t_.pose.position.y = 0;
        marker_6t_.pose.position.z = 0;
        marker_6t_.pose.orientation.x = 0.0;
        marker_6t_.pose.orientation.y = 0.0;
        marker_6t_.pose.orientation.z = sqrt(2) / 2;
        marker_6t_.pose.orientation.w = sqrt(2) / 2;
        marker_6t_.scale.x = scale_;
        marker_6t_.scale.y = scale_;
        marker_6t_.scale.z = scale_;
        marker_6t_.color.r = 0.0f;
        marker_6t_.color.g = 0.5f;
        marker_6t_.color.b = 0.0f;
        marker_6t_.color.a = 1.0;
        marker_6t_.lifetime = ros::Duration();
        marker_array_.markers.push_back(marker_6t_);
        // ros::spin();
        ros::Rate r(10);
        while (ros::ok())
        {
            while (marker_pub_.getNumSubscribers() < 1)
            {
                if (!ros::ok())
                {
                    return;
                }
                ROS_WARN_ONCE("Please create a subscriber to the marker");
                sleep(1);
            }
            marker_pub_.publish(marker_array_);
            // cur_marker_pub_.publish(marker_6t_);
            ros::spinOnce();
            r.sleep();
        }
    }
    void pointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        geometry_msgs::PoseStamped goal = *msg;

        marker_point_.header.frame_id = "map";
        marker_point_.header.stamp = ros::Time::now();
        marker_point_.ns = "clicked_points";
        marker_point_.id = 0;                                    // 确保每个标记有唯一 ID
        marker_point_.type = visualization_msgs::Marker::SPHERE; // 使用球体标记
        marker_point_.action = visualization_msgs::Marker::ADD;
        marker_point_.pose = goal.pose;
        marker_point_.scale.x = 1.0;
        marker_point_.scale.y = 1.0;
        marker_point_.scale.z = 1.0;
        marker_point_.color.r = 1.0f;
        marker_point_.color.g = 0.0f;
        marker_point_.color.b = 0.0f;
        marker_point_.color.a = 1.0;
        marker_point_.lifetime = ros::Duration(0); // 持续时间
        marker_array_.markers.push_back(marker_point_);

        marker_6t_.header.stamp = ros::Time::now();
        marker_6t_.ns = "pred";
        marker_6t_.id += 1; // 确保每个标记有唯一 ID
        marker_6t_.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker_6t_.mesh_resource = "file:///home/wxr/ivrc/6t/catkin_marker/src/base_link.dae";
        marker_6t_.action = visualization_msgs::Marker::ADD;
        marker_6t_.pose = goal.pose;
        marker_6t_.scale.x = scale_;
        marker_6t_.scale.y = scale_;
        marker_6t_.scale.z = scale_;
        marker_6t_.color.r = 0.0f;
        marker_6t_.color.g = 1.0f;
        marker_6t_.color.b = 0.0f;
        marker_6t_.color.a = pred_alpha_;
        marker_6t_.lifetime = ros::Duration(0); // 持续时间

        marker_array_.markers.push_back(marker_6t_);

        // marker_pub_.publish(marker_array_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber point_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher test_pub_;
    ros::Publisher cur_marker_pub_;
    visualization_msgs::Marker marker_6t_;
    visualization_msgs::Marker marker_point_;
    visualization_msgs::MarkerArray marker_array_;

    double scale_;
    double pred_alpha_;
    double vehicle_height_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clicked_point_subscriber");
    PointSubscriber point_subscriber;
    return 0;
}
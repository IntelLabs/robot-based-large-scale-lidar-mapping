/**
Copyright (C) 2025 Intel Corporation
SPDX-License-Identifier: Apache-2.0
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
 +File name: compute_normal.cpp
 +Description: This file contains the implementation of the ComputeNormal class, 
               which subscribes to a PointCloud2 topic, computes normals for the 
               point cloud, and publishes the result.
 +Author: Javier Felix-Rendon
 +Mail: javier.felix.rendon@intel.com
 +Version: 1.0
 +Date: 5/2/2025
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
*/

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <utility>
#include <vector>
#include <chrono>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>

// Using namespace for placeholders to simplify callback binding
using std::placeholders::_1;

// Define the ComputeNormal class, which inherits from rclcpp::Node
class ComputeNormal : public rclcpp::Node {
public:
    // Constructor: Initializes the node and sets up subscriptions and publishers
    ComputeNormal() : Node("compute_normal") {
        // Subscribe to the PointCloud2 topic to receive point cloud data
        subscription_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/scan/full", 10, 
            std::bind(&ComputeNormal::callback_scan, this, std::placeholders::_1)
        );

        // Publisher to publish the computed normals as a PointCloud2 message
        publisher_normal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar/scan/full_normals", 1
        );
    }

private:
    // Callback function to process incoming point cloud data
    void callback_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg_scan) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Computing normal");

        // Convert ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg_scan, *pclCloud);

        // Set up normal estimation
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
        normal_estimation.setInputCloud(pclCloud);

        // Use a KD-Tree for neighborhood search
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        normal_estimation.setSearchMethod(tree);

        // Compute normals for the point cloud
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        normal_estimation.setRadiusSearch(0.03); // Set the radius for normal computation
        normal_estimation.compute(*cloud_normals);

        // Combine the original point cloud with the computed normals
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::concatenateFields(*pclCloud, *cloud_normals, *outCloud);

        // Convert the combined cloud back to a ROS PointCloud2 message
        sensor_msgs::msg::PointCloud2 msg_normal;
        pcl::toROSMsg(*outCloud, msg_normal);
        msg_normal.header.frame_id = "tripod_floor_link"; // Set the frame ID for the message

        // Publish the message with computed normals
        publisher_normal_->publish(msg_normal);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Completado");
    }

    // Subscription to receive point cloud data
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_scan_;

    // Publisher to send the computed normals
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_normal_;
};

// Main function to initialize and run the node
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); // Initialize ROS 2

    // Create and spin the ComputeNormal node
    rclcpp::spin(std::make_shared<ComputeNormal>());

    rclcpp::shutdown(); // Shutdown ROS 2

    return 0;
}
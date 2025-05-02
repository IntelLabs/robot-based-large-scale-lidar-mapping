/**
Copyright (C) 2025 Intel Corporation
SPDX-License-Identifier: Apache-2.0
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
 +File name: l360_scan_node.cpp
 +Description: Aggregates 360-degree lidar scans into a single point cloud.
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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mapping_interfaces/msg/scan_step.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/msg/bool.hpp>
#include <mapping_interfaces/msg/scan_transformed_step.hpp>

using std::placeholders::_1;

// Global point cloud objects to store full, empty, and partial clouds
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_full_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_empty_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_partial_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

// Node class for processing 360-degree lidar scans
class L360ScanNode : public rclcpp::Node {
public:
    // Constructor: Initializes the node and sets up publishers and subscribers
    L360ScanNode() : Node("l360_scan_node") {
        // Subscription to transformed lidar scan messages
        subscription_scan_ = this->create_subscription<mapping_interfaces::msg::ScanTransformedStep>(
            "/lidar/scan/transformed", 10, 
            std::bind(&L360ScanNode::callback_scan, this, std::placeholders::_1)
        );
        // Publisher for the aggregated full point cloud
        publisher_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/scan/full", 1);
        // Publisher to signal the completion of a scan step
        publisher_finish_step_ = this->create_publisher<std_msgs::msg::Bool>("/finish_step", 1);
    }

private:
    // Callback function to process incoming scan messages
    void callback_scan(const mapping_interfaces::msg::ScanTransformedStep::SharedPtr scan_msg) const {
        // Extract the point cloud from the message
        sensor_msgs::msg::PointCloud2 cloud_msg = scan_msg->cloud;
        pcl_partial_cloud->clear(); // Clear the partial cloud
        pcl_partial_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>); // Reset the partial cloud
        pcl::fromROSMsg(cloud_msg, *pcl_partial_cloud); // Convert ROS message to PCL point cloud

        int scan_step = scan_msg->step; // Get the current scan step

        // Aggregate the partial cloud into the full cloud for steps 1 to 8
        if (scan_step > 0 && scan_step <= 8) {
            pcl::concatenate(*pcl_full_cloud, *pcl_partial_cloud, *pcl_full_cloud);
        }

        // If the scan step is the last one (8), publish the full cloud
        if (scan_step == 8) {
            sensor_msgs::msg::PointCloud2 full_cloud;
            pcl::toROSMsg(*pcl_full_cloud, full_cloud); // Convert PCL point cloud to ROS message
            full_cloud.header.frame_id = "tripod_floor_link"; // Set the frame ID
            publisher_cloud_->publish(full_cloud); // Publish the full cloud
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "published"); // Log the publication
            pcl_full_cloud->clear(); // Clear the full cloud
            pcl_full_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>); // Reset the full cloud
        }

        // Publish a message indicating the completion of the scan step
        std_msgs::msg::Bool finish_s_msg;
        finish_s_msg.data = true;
        publisher_finish_step_->publish(finish_s_msg);
    }

    // Subscriber for transformed scan messages
    rclcpp::Subscription<mapping_interfaces::msg::ScanTransformedStep>::SharedPtr subscription_scan_;
    // Publisher for the aggregated full point cloud
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_cloud_;
    // Publisher to signal the completion of a scan step
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_finish_step_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); // Initialize ROS 2

    rclcpp::spin(std::make_shared<L360ScanNode>()); // Spin the node

    rclcpp::shutdown(); // Shutdown ROS 2

    return 0;
}
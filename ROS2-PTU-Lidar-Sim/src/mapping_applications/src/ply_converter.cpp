/**
Copyright (C) 2025 Intel Corporation
SPDX-License-Identifier: Apache-2.0
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
 +File name: ply_converter.cpp
 +Description: Converts lidar point cloud data to PLY format with optional normals.
 +Author: Javier Felix-Rendon
 +Mail: javier.felix.rendon@intel.com
 +Version: 1.0
 +Date: 5/2/2025
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

// Global variables for parameters and PTU angles
int param_normals_; // Determines if normals are included in the PLY file
int save_ply_;      // Determines if PLY files should be saved
float pan_angle, tilt_angle; // PTU pan and tilt angles
geometry_msgs::msg::Pose model_pose; // Pose of the lidar model

// Namespace alias for PCL point cloud types
namespace my_pcl_namespace {
  using PointXYZRGBNormal = pcl::PointXYZRGBNormal;
  using PointCloudXYZRGBNormal = pcl::PointCloud<PointXYZRGBNormal>;
}

// Main class for converting point cloud data to PLY format
class PointCloudToPLY : public rclcpp::Node {
public:
  PointCloudToPLY() : Node("pointcloud_to_ply_node"), scan_count_(0) {
    // Declare and retrieve parameters
    declare_parameter("param_normals", 1);
    get_parameter("param_normals", param_normals_);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "param_normals %d", param_normals_);

    declare_parameter("save_ply", 1);
    get_parameter("save_ply", save_ply_);

    // Subscribe to the appropriate topic based on the 'param_normals' parameter
    if (param_normals_ == 1) {
      subscription_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar/scan/full_normals", 10, std::bind(&PointCloudToPLY::cloudCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Subscribed to '/lidar/scan/full_normals' topic.");
    } else {
      subscription_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar/scan/full", 10, std::bind(&PointCloudToPLY::cloudCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Subscribed to '/lidar/scan/full' topic.");
    }

    // Ensure the 'ply' directory exists
    if (!std::filesystem::exists("ply")) {
      if (std::filesystem::create_directory("ply")) {
        RCLCPP_INFO(this->get_logger(), "Created 'ply' directory.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to create 'ply' directory.");
      }
    }

    // Subscribe to PTU and model pose topics
    subscription_pan_angle_ = this->create_subscription<std_msgs::msg::Float64>(
      "/pan_pos_controller", 10, std::bind(&PointCloudToPLY::callback_pan_angle, this, std::placeholders::_1));

    subscription_tilt_angle_ = this->create_subscription<std_msgs::msg::Float64>(
      "/tilt_pos_controller", 10, std::bind(&PointCloudToPLY::callback_tilt_angle, this, std::placeholders::_1));

    subscription_model_pose_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/set_ptu_position", 10, std::bind(&PointCloudToPLY::callback_model_pose, this, std::placeholders::_1));
  }

private:
  // Callback for pan angle updates
  void callback_pan_angle(std_msgs::msg::Float64::SharedPtr msg_pan) {
    pan_angle = msg_pan->data;
  }

  // Callback for tilt angle updates
  void callback_tilt_angle(std_msgs::msg::Float64::SharedPtr msg_tilt) {
    tilt_angle = msg_tilt->data;
  }

  // Callback for model pose updates
  void callback_model_pose(const geometry_msgs::msg::Pose::SharedPtr msg_model_pose) {
    model_pose.position.x = msg_model_pose->position.x;
    model_pose.position.y = msg_model_pose->position.y;
    model_pose.position.z = msg_model_pose->position.z;
    model_pose.orientation.x = msg_model_pose->orientation.x;
    model_pose.orientation.y = msg_model_pose->orientation.y;
    model_pose.orientation.z = msg_model_pose->orientation.z;
    model_pose.orientation.w = msg_model_pose->orientation.w;
  }

  // Callback for processing incoming point cloud data
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (save_ply_ == 1) {
      RCLCPP_INFO(this->get_logger(), "Received a PointCloud2 message.");

      // Convert ROS 2 PointCloud2 message to PCL PointCloud
      my_pcl_namespace::PointCloudXYZRGBNormal::Ptr pcl_cloud(new my_pcl_namespace::PointCloudXYZRGBNormal);
      pcl::fromROSMsg(*msg, *pcl_cloud);

      // Filter out invalid points (NaN or Inf values)
      my_pcl_namespace::PointCloudXYZRGBNormal::Ptr filtered_cloud(new my_pcl_namespace::PointCloudXYZRGBNormal);
      for (const auto& point : pcl_cloud->points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
          filtered_cloud->points.push_back(point);
        }
      }
      filtered_cloud->width = filtered_cloud->points.size();
      filtered_cloud->height = 1; // Unorganized point cloud
      filtered_cloud->is_dense = true;

      // Generate the filename for the PLY file
      std::string filename;
      if (param_normals_ == 1) {
        filename = "ply_normals/scan_" + std::to_string(scan_count_) + ".ply";
      } else {
        filename = "ply/scan_" + std::to_string(scan_count_) + ".ply";
      }

      // Save the filtered point cloud to a PLY file
      if (pcl::io::savePLYFileBinary(filename, *filtered_cloud) == 0) {
        // Add metadata comments to the PLY file
        std::ifstream inputFile(filename);
        std::string first_line;
        std::getline(inputFile, first_line);
        std::string second_line;
        std::getline(inputFile, second_line);

        std::stringstream buffer;
        buffer << inputFile.rdbuf();
        std::string content = buffer.str();

        inputFile.close();

        std::ofstream outputFile(filename);
        outputFile << first_line << std::endl;
        outputFile << second_line << std::endl;
        outputFile << "comment pan angle " << pan_angle << "\n";
        outputFile << "comment tilt angle " << tilt_angle << "\n";
        outputFile << "comment lidar position x " << model_pose.position.x << "\n";
        outputFile << "comment lidar position y " << model_pose.position.y << "\n";
        outputFile << "comment lidar position z " << model_pose.position.z << "\n";
        outputFile << "comment lidar orientation x " << model_pose.orientation.x << "\n";
        outputFile << "comment lidar orientation y " << model_pose.orientation.y << "\n";
        outputFile << "comment lidar orientation z " << model_pose.orientation.z << "\n";
        outputFile << "comment lidar orientation w " << model_pose.orientation.w << "\n";
        outputFile << content;
        outputFile.close();

        scan_count_++; // Increment the scan count
        RCLCPP_INFO(this->get_logger(), "Saved point cloud to %s", filename.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s", filename.c_str());
      }
    }
  }

  // Subscriptions for point cloud, PTU angles, and model pose
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_scan_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_pan_angle_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_tilt_angle_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_model_pose_;
  int scan_count_; // Counter for saved scans
};

// Main function to initialize and run the node
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToPLY>());
  rclcpp::shutdown();
  return 0;
}
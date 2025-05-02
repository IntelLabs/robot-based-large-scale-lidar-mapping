/**
 Copyright (C) 2025 Intel Corporation
SPDX-License-Identifier: Apache-2.0
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
 +File name: world_scan_server.cpp
 +Description: Implements a ROS 2 node for scanning the world using a PTU and lidar system.
 +Author: Javier Felix-Rendon
 +Mail: javier.felix.rendon@intel.com
 +Version: 1.0
 +Date: 5/2/2025
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
 */

#include "rclcpp/rclcpp.hpp"
#include "mapping_interfaces/srv/world_scan.hpp"
#include "mapping_interfaces/msg/scan_step.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <math.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

// Global variables for scan dimensions and PTU control
geometry_msgs::msg::Vector3 dimensions; // Stores the x and y dimensions of the scan area
bool lidar_collision = false;           // Flag to indicate if a collision is detected

// PTU angle limits
float pan_min_angle = -M_PI;  // Minimum pan angle
float pan_max_angle = M_PI;   // Maximum pan angle
float tilt_min_angle = -1.2;  // Minimum tilt angle
float tilt_max_angle = 0.5;   // Maximum tilt angle

// Timing variables
float estimated_time;         // Estimated time for the scan
int wait_time_1 = 3000;       // Wait time in milliseconds for lidar scanning
int wait_time_2 = 4000;       // Wait time in milliseconds for collision checks

bool finish_s = false;        // Flag to indicate if a scan step is finished
int update_collision = 0;     // Counter for collision updates

class WorldScanServer : public rclcpp::Node {
public:
    WorldScanServer() : Node("world_scan_server") {
        // Create a service to handle world scan requests
        service_ = this->create_service<mapping_interfaces::srv::WorldScan>(
            "world_scan", std::bind(&WorldScanServer::handle_service, this, _1, std::placeholders::_2));

        // Publishers for PTU and lidar control
        publisher_model_position_ = this->create_publisher<geometry_msgs::msg::Pose>("/set_ptu_position", 1);
        publisher_step_scan_ = this->create_publisher<mapping_interfaces::msg::ScanStep>("/step_scan", 1);
        publisher_pan_controller_ = this->create_publisher<std_msgs::msg::Float64>("/pan_pos_controller", 1);
        publisher_tilt_controller_ = this->create_publisher<std_msgs::msg::Float64>("/tilt_pos_controller", 1);
        publisher_lidar_controller_ = this->create_publisher<std_msgs::msg::Float64>("/lidar_pos_controller", 1);
    }

private:
    // Service and publishers
    rclcpp::Service<mapping_interfaces::srv::WorldScan>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_model_position_;
    rclcpp::Publisher<mapping_interfaces::msg::ScanStep>::SharedPtr publisher_step_scan_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_pan_controller_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_tilt_controller_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_lidar_controller_;

    void handle_service(const std::shared_ptr<mapping_interfaces::srv::WorldScan::Request> request,
                        std::shared_ptr<mapping_interfaces::srv::WorldScan::Response> response) {
        // Handle incoming world scan requests
        int time_count = 0; // Counter for elapsed time
        float XGridSize = request->x_grid_size; // Grid size in x-direction
        float YGridSize = request->y_grid_size; // Grid size in y-direction
        dimensions.x = request->x_dimension;    // Total x-dimension of the scan area
        dimensions.y = request->y_dimension;    // Total y-dimension of the scan area

        // Log the dimensions of the scan area
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x_dim %f   y_dim %f", dimensions.x, dimensions.y);

        // Calculate the number of steps in x and y directions
        int x_steps = dimensions.x / XGridSize;
        int y_steps = dimensions.y / YGridSize;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x_steps %d   y_steps %d", x_steps, y_steps);

        // Initialize the starting position for the scan
        float x_pose = -(dimensions.x / 2);
        float y_pose = -(dimensions.y / 2);

        geometry_msgs::msg::Pose msg_position; // Message to set PTU position

        // Retrieve pan and tilt steps from the request
        int pan_steps = request->pan_steps;
        int tilt_steps = request->tilt_steps;

        // Calculate step angles for pan, tilt, and lidar
        float pan_step_angle = (abs(pan_min_angle) + abs(pan_max_angle)) / pan_steps;
        float tilt_step_angle = (abs(tilt_min_angle) + abs(tilt_max_angle)) / tilt_steps;
        float lidar_step_angle = 0.785399; // 45 degrees

        float pan_s = pan_min_angle; // Starting pan angle
        float tilt_s = tilt_min_angle; // Starting tilt angle
        float lidar_s = 0; // Starting lidar angle

        int lidar_steps = 8; // Number of lidar steps (1 step per camera)

        // Log PTU angle limits and step angles
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pan_min_angle %f   pan_max_angle %f", pan_min_angle, pan_max_angle);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "tilt_min_angle %f   tilt_max_angle %f", tilt_min_angle, tilt_max_angle);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pan_steps %d   tilt_steps %d", pan_steps, tilt_steps);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pan_steps_angle %f   tilt_step_angle %f", pan_step_angle, tilt_step_angle);

        // Calculate the total number of scans and estimated time
        int total_scans = x_steps * y_steps * pan_steps * tilt_steps;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Number of scans: %d", total_scans);

        estimated_time = x_steps * y_steps * pan_steps * tilt_steps * ((float(wait_time_1) / 1000) + (float(wait_time_2) / 1000)) * lidar_steps;

        int hours = floor(estimated_time / 3600);
        int minutes = floor((fmod(estimated_time, 3600)) / 60);
        int seconds = floor(fmod(estimated_time, 60));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Estimated time: %f", estimated_time);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Estimated time: %d hr, %d min, %d sec", hours, minutes, seconds);

        for (int s = 1; s <= 2; s++) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Check if PTU-Lidar cover the 4 edges of the model to scan, if not preess ctrl+c and adjust x-dimension and y-dimension parameters");

            msg_position.position.x = -(dimensions.x / 2);
            msg_position.position.y = -(dimensions.y / 2);
            msg_position.position.z = 0.0;
            msg_position.orientation.w = 1.0;
            msg_position.orientation.x = 0.0;
            msg_position.orientation.y = 0.0;
            msg_position.orientation.z = 0.0;

            publisher_model_position_->publish(msg_position);
            std::this_thread::sleep_for(std::chrono::seconds(1));

            msg_position.position.x = -(dimensions.x / 2);
            msg_position.position.y = (dimensions.y / 2);
            publisher_model_position_->publish(msg_position);
            std::this_thread::sleep_for(std::chrono::seconds(1));

            msg_position.position.x = (dimensions.x / 2);
            msg_position.position.y = -(dimensions.y / 2);
            publisher_model_position_->publish(msg_position);
            std::this_thread::sleep_for(std::chrono::seconds(1));

            msg_position.position.x = (dimensions.x / 2);
            msg_position.position.y = (dimensions.y / 2);
            publisher_model_position_->publish(msg_position);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        x_pose = -(dimensions.x / 2);

        for (int i = 1; i <= x_steps; i++) {
            for (int j = 1; j <= y_steps; j++) {
                msg_position.position.x = x_pose;
                msg_position.position.y = y_pose;
                msg_position.position.z = 0.0;
                msg_position.orientation.w = 1.0;
                msg_position.orientation.x = 0.0;
                msg_position.orientation.y = 0.0;
                msg_position.orientation.z = 0.0;

                publisher_model_position_->publish(msg_position);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                update_collision = 0;

                while (update_collision > 10000) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_2));
                }

                std_msgs::msg::Float64 msg_pan_pos;
                std_msgs::msg::Float64 msg_tilt_pos;
                std_msgs::msg::Float64 msg_lidar_pos;

                if (lidar_collision == false) {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lidar ready to scan on point %f %f", x_pose, y_pose);

                    pan_s = pan_min_angle;
                    tilt_s = tilt_min_angle;

                    for (int m = 1; m <= pan_steps; m++) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pan_step %d ", m);
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pan_angle %f ", pan_s);

                        for (int n = 1; n <= tilt_steps; n++) {
                            msg_lidar_pos.data = lidar_s;
                            msg_pan_pos.data = pan_s;
                            msg_tilt_pos.data = tilt_s;

                            publisher_lidar_controller_->publish(msg_lidar_pos);
                            publisher_pan_controller_->publish(msg_pan_pos);
                            publisher_tilt_controller_->publish(msg_tilt_pos);

                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "tilt_step %d ", n);
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "tilt_angle %f ", tilt_s);

                            mapping_interfaces::msg::ScanStep msg_step;

                            float lidar_s = 0;

                            for (int steps_360scan = 1; steps_360scan <= lidar_steps; steps_360scan++) {
                                msg_lidar_pos.data = lidar_s;

                                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Step_Lidar %d ", steps_360scan);

                                publisher_lidar_controller_->publish(msg_lidar_pos);
                                publisher_pan_controller_->publish(msg_pan_pos);
                                publisher_tilt_controller_->publish(msg_tilt_pos);

                                std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_1));

                                msg_step.scanstep = steps_360scan;
                                publisher_step_scan_->publish(msg_step);

                                while (finish_s == false) {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_2));
                                }

                                time_count++;

                                float time_left = estimated_time - (time_count * ((float(wait_time_1) / 1000) + (float(wait_time_2) / 1000)));

                                int lhours = floor(time_left / 3600);
                                int lminutes = floor((fmod(time_left, 3600)) / 60);
                                int lseconds = floor(fmod(time_left, 60));
                                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left estimated time: %f ", time_left);
                                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left estimated time: %d hr, %d min, %d sec", lhours, lminutes, lseconds);

                                lidar_s = lidar_s + lidar_step_angle;
                                finish_s = false;
                            }

                            tilt_s = tilt_s + tilt_step_angle;
                        }

                        pan_s = pan_s + pan_step_angle;
                        tilt_s = tilt_min_angle;
                    }

                    pan_s = pan_min_angle;

                } else {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lidar cannot be placed on point %f %f", x_pose, y_pose);

                    time_count = time_count + (pan_steps * tilt_steps * 8);
                }

                y_pose = y_pose + YGridSize;
            }

            y_pose = -(dimensions.y / 2);
            x_pose = x_pose + XGridSize;
        }
    }
};

class SubsNode : public rclcpp::Node {
public:
    SubsNode() : Node("subs_node") {
        // Subscribe to PTU collision and finish step topics
        subscription_ptu_collision_ = this->create_subscription<std_msgs::msg::Bool>(
            "/ptu_collision", 1, std::bind(&SubsNode::callback_subs_ptu_collision, this, _1));
        finish_step_ = this->create_subscription<std_msgs::msg::Bool>(
            "/finish_step", 1, std::bind(&SubsNode::callback_subs_finish_step, this, _1));
    }

private:
    void callback_subs_ptu_collision(const std_msgs::msg::Bool::SharedPtr msg) {
        // Callback to handle PTU collision updates
        lidar_collision = msg->data;
        update_collision++;
    }

    void callback_subs_finish_step(const std_msgs::msg::Bool::SharedPtr msg) {
        // Callback to handle finish step updates
        finish_s = msg->data;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Finish step ");
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_ptu_collision_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finish_step_;
};

int main(int argc, char **argv) {
    // Main function to initialize ROS 2 nodes and spin the executor
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WorldScanServer>();
    auto subs_node = std::make_shared<SubsNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(subs_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
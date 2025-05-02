/**
Copyright (C) 2025 Intel Corporation
SPDX-License-Identifier: Apache-2.0
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
 +File name: world_scan_client.cpp
 +Description: Implements a ROS 2 client node to request world scanning services.
 +Author: Javier Felix-Rendon
 +Mail: javier.felix.rendon@intel.com
 +Version: 1.0
 +Date: 5/2/2025
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
*/
#include "rclcpp/rclcpp.hpp"
#include "mapping_interfaces/srv/world_scan.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

// Global variables to store parameters for the world scan
float param_x_grid_size_, param_y_grid_size_, param_pan_steps_, param_tilt_steps_, x_dimension_, y_dimension_;

class WorldScanClient : public rclcpp::Node {
public:
    WorldScanClient() : Node("world_scan_client") {
        // Declare and initialize parameters for the world scan
        declare_parameter<double>("param_x_grid_size", 1.0);  // Grid size in the x-direction
        declare_parameter<double>("param_y_grid_size", 1.0);  // Grid size in the y-direction
        declare_parameter<double>("param_pan_steps", 10.0);   // Number of pan steps
        declare_parameter<double>("param_tilt_steps", 10.0);  // Number of tilt steps
        declare_parameter<double>("x_dimension", 50.0);       // Total x-dimension of the scan area
        declare_parameter<double>("y_dimension", 30.0);       // Total y-dimension of the scan area

        // Retrieve parameter values
        get_parameter("param_x_grid_size", param_x_grid_size_);
        get_parameter("param_y_grid_size", param_y_grid_size_);
        get_parameter("param_pan_steps", param_pan_steps_);
        get_parameter("param_tilt_steps", param_tilt_steps_);
        get_parameter("x_dimension", x_dimension_);
        get_parameter("y_dimension", y_dimension_);

        // Create a client to interact with the "world_scan" service
        client = this->create_client<mapping_interfaces::srv::WorldScan>("world_scan");

        // Create a request object and populate it with parameter values
        auto request = std::make_shared<mapping_interfaces::srv::WorldScan::Request>();
        request->x_grid_size = param_x_grid_size_;
        request->y_grid_size = param_y_grid_size_;
        request->pan_steps = param_pan_steps_;
        request->tilt_steps = param_tilt_steps_;
        request->x_dimension = x_dimension_;
        request->y_dimension = y_dimension_;

        // Wait for the service to become available
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        // Send the request to the service asynchronously
        auto result = client->async_send_request(std::move(request));

        // Wait for the result of the service call
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "World Scan Complete");  // Success message
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service world_scan");  // Error message
        }
    }

private:
    // Client to interact with the "world_scan" service
    rclcpp::Client<mapping_interfaces::srv::WorldScan>::SharedPtr client;
};

int main(int argc, char *argv[]) {
    // Main function to initialize the ROS 2 node and spin the client
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WorldScanClient>());
    rclcpp::shutdown();
    return 0;
}
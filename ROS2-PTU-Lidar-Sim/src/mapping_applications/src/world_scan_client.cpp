#include "rclcpp/rclcpp.hpp"
#include "mapping_interfaces/srv/world_scan.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

float param_x_grid_size_,param_y_grid_size_,param_pan_steps_,param_tilt_steps_,x_dimension_,y_dimension_;



class WorldScanClient : public rclcpp::Node
{
  public:
    WorldScanClient() : Node("world_scan_client")
    {

      declare_parameter<double>("param_x_grid_size", 1.0);
      declare_parameter<double>("param_y_grid_size", 1.0);
      declare_parameter<double>("param_pan_steps", 10.0);
      declare_parameter<double>("param_tilt_steps", 10.0);
      declare_parameter<double>("x_dimension", 50.0);
      declare_parameter<double>("y_dimension", 30.0);

      get_parameter("param_x_grid_size", param_x_grid_size_);
      get_parameter("param_y_grid_size", param_y_grid_size_);
      get_parameter("param_pan_steps", param_pan_steps_);
      get_parameter("param_tilt_steps", param_tilt_steps_);
      get_parameter("x_dimension", x_dimension_);
      get_parameter("y_dimension", y_dimension_);


     
        client = this->create_client<mapping_interfaces::srv::WorldScan>("world_scan");


         auto request = std::make_shared<mapping_interfaces::srv::WorldScan::Request>();
        request->x_grid_size = param_x_grid_size_;
        request->y_grid_size = param_y_grid_size_;
        request->pan_steps = param_pan_steps_;
        request->tilt_steps = param_tilt_steps_;
        request->x_dimension = x_dimension_;
        request->y_dimension = y_dimension_;

        while (!client->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          }
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = client->async_send_request(std::move(request));

        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
          rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "World Scan Complete");
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service world_scan");
        }
      
    }

  private:
    



    

   
    rclcpp::Client<mapping_interfaces::srv::WorldScan>::SharedPtr client;

    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<WorldScanClient>());



  rclcpp::shutdown();



  return 0;
}
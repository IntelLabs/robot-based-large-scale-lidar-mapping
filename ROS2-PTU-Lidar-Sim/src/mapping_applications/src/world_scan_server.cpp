#include "rclcpp/rclcpp.hpp"
#include "mapping_interfaces/srv/world_scan.hpp"
// #include "interfaces/srv/world_dim.hpp"
// #include "interfaces/srv/model_pose.hpp"
#include "mapping_interfaces/msg/scan_step.hpp"
// #include "interfaces/msg/pos_lidar.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
//#include <gazebo_ros/conversions/geometry_msgs.hpp>
//#include <gazebo_msgs/srv/spawn_entity.hpp>



#include <memory>
#include <string>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <math.h>






using std::placeholders::_1;
using namespace std::chrono_literals;

geometry_msgs::msg::Vector3 dimensions;
bool lidar_collision = false;

float pan_min_angle= -M_PI;
float pan_max_angle= M_PI;
float tilt_min_angle=-1.2;
float tilt_max_angle=0.5;

float estimated_time;
int wait_time_1 = 1000;
int wait_time_2 = 1000;

bool finish_s = false;
int update_collision = 0;


class WorldScanServer : public rclcpp::Node
{
  public:
    WorldScanServer() : Node("world_scan_server")
    {

    service_ = this->create_service<mapping_interfaces::srv::WorldScan>(
        "world_scan", std::bind(&WorldScanServer::handle_service, this,
                                  std::placeholders::_1, std::placeholders::_2));

    //subscription_dims_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    //  "/world_dimension", 10, std::bind(&WorldScanServer::callback_subs, this, _1));

    publisher_model_position_ = this->create_publisher<geometry_msgs::msg::Pose>("/set_ptu_position",1);

    //publisher_lidar_position_ = this->create_publisher<interfaces::msg::PosLidar>("/controller_ptu/set_pos_lidar",1);

    publisher_step_scan_ = this->create_publisher<mapping_interfaces::msg::ScanStep>("/step_scan",1);

    //publisher_step_cam_ = this->create_publisher<std_msgs::msg::Int16>("/step_cam",1);

    publisher_pan_controller_ = this->create_publisher<std_msgs::msg::Float64>("/pan_pos_controller",1);

    publisher_tilt_controller_ = this->create_publisher<std_msgs::msg::Float64>("/tilt_pos_controller",1);
    
    publisher_lidar_controller_ = this->create_publisher<std_msgs::msg::Float64>("/lidar_pos_controller",1);


    }



private:
  rclcpp::Service<mapping_interfaces::srv::WorldScan>::SharedPtr service_;
  //rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_dims_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_model_position_;
  //rclcpp::Publisher<interfaces::msg::PosLidar>::SharedPtr publisher_lidar_position_;
  rclcpp::Publisher<mapping_interfaces::msg::ScanStep>::SharedPtr publisher_step_scan_;
 // rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_step_cam_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_pan_controller_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_tilt_controller_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_lidar_controller_;


  // void callback_subs(const geometry_msgs::msg::Vector3::SharedPtr msg)
  // {
  //   dimensions.x=msg->x;
  //   dimensions.y=msg->y;
  //   dimensions.z=msg->z;
  //   //RCLCPP_INFO(this->get_logger(), "Received response subs dims");
  // }


void handle_service(const std::shared_ptr<mapping_interfaces::srv::WorldScan::Request> request,
          std::shared_ptr<mapping_interfaces::srv::WorldScan::Response>      response)
		{
			int time_count = 0;

      float XGridSize = request->x_grid_size;
      float YGridSize = request->y_grid_size;

      dimensions.x = request->x_dimension;
      dimensions.y = request->y_dimension;

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x_dim %f   y_dim %f", dimensions.x,dimensions.y);

      //XGridSize = 2.0;
      //YGridSize = 2.0;

      int x_steps = dimensions.x/XGridSize;
      int y_steps = dimensions.y/YGridSize;

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x_steps %d   y_steps %d", x_steps,y_steps);

      float x_pose= -(dimensions.x/2);
      float y_pose= -(dimensions.y/2);

      geometry_msgs::msg::Pose msg_position;

      int pan_steps = request->pan_steps;
      int tilt_steps = request->tilt_steps;

      //pan_step_sz = 10.0;
      //tilt_step_sz = 10.0;

      
      float pan_step_angle = (abs(pan_min_angle)+ abs(pan_max_angle))/pan_steps;
      float tilt_step_angle = (abs(tilt_min_angle)+ abs(tilt_max_angle))/tilt_steps;
      float lidar_step_angle = 0.785399; //45 degrees

      float pan_s = pan_min_angle;
      float tilt_s = tilt_min_angle;
      float lidar_s = 0;

      int lidar_steps = 8;   //1 step per camera


      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pan_steps %d ", pan_steps);   //ARREGLAR PAN STEPS x_steps, y_steps, MODIFICADO PARA OBTENER DATOS
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pan_min_angle %f   pan_max_angle %f", pan_min_angle,pan_max_angle);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "tilt_min_angle %f   tilt_max_angle %f", tilt_min_angle,tilt_max_angle);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pan_steps %d   tilt_steps %d", pan_steps,tilt_steps);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pan_steps_angle %f   tilt_step_angle %f", pan_step_angle,tilt_step_angle);

      int total_scans = x_steps * y_steps * pan_steps * tilt_steps;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Number of scans: %d", total_scans);

      estimated_time = x_steps * y_steps * pan_steps * tilt_steps * ((float(wait_time_1)/1000) + (float(wait_time_2)/1000)) * lidar_steps;

      int hours = floor(estimated_time / 3600);
      int minutes = floor((fmod(estimated_time,3600)) / 60);
      int seconds = floor(fmod(estimated_time,60));
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Estimated time: %f", estimated_time);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Estimated time: %d hr, %d min, %d sec", hours,minutes,seconds);



      for (int s=1; s<=2; s++){

          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Check if PTU-Lidar cover the 4 edges of the model to scan, if not preess ctrl+c and adjust x-dimension and y-dimension parameters");

          msg_position.position.x=-(dimensions.x/2);;
          msg_position.position.y=-(dimensions.y/2);;
          msg_position.position.z=0.0;
          msg_position.orientation.w=1.0;
          msg_position.orientation.x=0.0;
          msg_position.orientation.y=0.0;
          msg_position.orientation.z=0.0;

          publisher_model_position_->publish(msg_position);

          std::this_thread::sleep_for(std::chrono::seconds(1));

          msg_position.position.x=-(dimensions.x/2);;
          msg_position.position.y=(dimensions.y/2);;
          

          publisher_model_position_->publish(msg_position);

          std::this_thread::sleep_for(std::chrono::seconds(1));

          msg_position.position.x=(dimensions.x/2);;
          msg_position.position.y=-(dimensions.y/2);;
          

          publisher_model_position_->publish(msg_position);

          std::this_thread::sleep_for(std::chrono::seconds(1));

          msg_position.position.x=(dimensions.x/2);;
          msg_position.position.y=(dimensions.y/2);;
          

          publisher_model_position_->publish(msg_position);

          std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      x_pose=-(dimensions.x/2);
      
      for (int i = 1; i <= x_steps; i++) {
          

        for (int j = 1; j <= y_steps; j++) {
        



          msg_position.position.x=x_pose;
          msg_position.position.y=y_pose;
          msg_position.position.z=0.0;
          msg_position.orientation.w=1.0;
          msg_position.orientation.x=0.0;
          msg_position.orientation.y=0.0;
          msg_position.orientation.z=0.0;

          publisher_model_position_->publish(msg_position);

          std::this_thread::sleep_for(std::chrono::seconds(1));


          update_collision = 0;

          while (update_collision>10000)
            {
              std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_2));
            }


          //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Here model orientation %f ",model_orientation_w);


          std_msgs::msg::Float64 msg_pan_pos;
          std_msgs::msg::Float64 msg_tilt_pos;
          std_msgs::msg::Float64 msg_lidar_pos;

          if (lidar_collision == false){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lidar ready to scan on point %f %f",x_pose,y_pose);
            

          /*
            std_msgs::msg::Int16 msg_step_cam;
            for (int m = 0; m<= pan_steps; m++){   //For used to save image in 360 degrees
              msg_lidar_pos.pos_lidar=0;
              msg_lidar_pos.pos_pan=pan_s;
              msg_lidar_pos.pos_tilt=0;

              publisher_lidar_position_->publish(msg_lidar_pos);
              //publisher_model_position_->publish(msg_position);

              std::this_thread::sleep_for(std::chrono::seconds(3));

              msg_step_cam.data=m;
              publisher_step_cam_->publish(msg_step_cam);

              std::this_thread::sleep_for(std::chrono::seconds(3));

              pan_s = pan_s + pan_step_sz;
            }
            */
            pan_s = pan_min_angle;
            tilt_s = tilt_min_angle;

            for (int m = 1; m<= pan_steps; m++){
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pan_step %d ", m);
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pan_angle %f ", pan_s);

              
              for (int n = 1; n <= tilt_steps; n++){    

                  msg_lidar_pos.data=lidar_s;
                  msg_pan_pos.data=pan_s;
                  msg_tilt_pos.data=tilt_s;

                  publisher_lidar_controller_->publish(msg_lidar_pos);
                  publisher_pan_controller_ ->publish(msg_pan_pos);
                  publisher_tilt_controller_ ->publish(msg_tilt_pos);

                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "tilt_step %d ", n);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "tilt_angle %f ", tilt_s);
                 


                  

                  mapping_interfaces::msg::ScanStep msg_step;
                  
                  float lidar_s = 0;

                  for (int steps_360scan=1; steps_360scan<=lidar_steps; steps_360scan++){  

                     msg_lidar_pos.data=lidar_s;

                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Step_Lidar %d ", steps_360scan);

                    publisher_lidar_controller_->publish(msg_lidar_pos);
                    publisher_pan_controller_ ->publish(msg_pan_pos);
                    publisher_tilt_controller_ ->publish(msg_tilt_pos);

                    std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_1));
                   
                    


                    msg_step.scanstep=steps_360scan;
                    publisher_step_scan_->publish(msg_step);

                    
                    while (finish_s==false)
                    {
                      std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_2));
                    }
                    

                    
                    time_count ++;

                    float time_left = estimated_time - (time_count*((float(wait_time_1)/1000)+(float(wait_time_2)/1000)));

                    int lhours = floor(time_left / 3600);
                    int lminutes = floor((fmod(time_left,3600)) / 60);
                    int lseconds = floor(fmod(time_left,60));
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left estimated time: %f ", time_left);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left estimated time: %d hr, %d min, %d sec", lhours,lminutes,lseconds);

                    lidar_s=lidar_s+lidar_step_angle;
                    finish_s=false;
                     
                  }

                  tilt_s = tilt_s + tilt_step_angle;
              }

              pan_s = pan_s + pan_step_angle;
              tilt_s = tilt_min_angle;

            }

            pan_s = pan_min_angle; 



          }
          else{
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lidar cannot be placed on point %f %f",x_pose,y_pose);

            time_count = time_count + (pan_steps * tilt_steps * 8);

          }






          y_pose=y_pose+YGridSize;
        }

        y_pose=-(dimensions.y/2);
        x_pose=x_pose+XGridSize;

      }
      

		}

};









class SubsNode : public rclcpp::Node
{
  public:
    SubsNode() : Node("subs_node")
    {
      subscription_ptu_collision_ = this->create_subscription<std_msgs::msg::Bool>(
      "/ptu_collision", 1, std::bind(&SubsNode::callback_subs_ptu_collision, this, _1));

      finish_step_ = this->create_subscription<std_msgs::msg::Bool>(
      "/finish_step", 1, std::bind(&SubsNode::callback_subs_finish_step, this, _1));

    }

  private:




    void callback_subs_ptu_collision(const std_msgs::msg::Bool::SharedPtr msg)
  {
    lidar_collision = msg->data;
    update_collision++;
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received model orientation %f ",model_orientation_w);
  }

    void callback_subs_finish_step(const std_msgs::msg::Bool::SharedPtr msg)
  {
    finish_s = msg->data;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Finish step ");
  }




  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_ptu_collision_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finish_step_;
};




int main(int argc, char **argv)
{
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
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


pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_full_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_empty_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_partial_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//sensor_msgs::msg::PointCloud2 full_cloud;



class L360ScanNode : public rclcpp::Node
{
  public:
    L360ScanNode() : Node("l360_scan_node")
    {
      subscription_scan_ = this->create_subscription<mapping_interfaces::msg::ScanTransformedStep>(
      "/lidar/scan/transformed", 10, std::bind(&L360ScanNode::callback_scan, this, std::placeholders::_1));
      publisher_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/scan/full",1);
      publisher_finish_step_ = this->create_publisher<std_msgs::msg::Bool>("/finish_step",1);
    }

  private:
    
    // void callback_scan(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) const
    // {
    //   /*
    //   pcl_full_cloud->clear();
    //   pcl_full_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    //   pcl::fromROSMsg(*cloud_msg,*pcl_full_cloud);
             
    //   pcl::toROSMsg(*pcl_full_cloud,full_cloud);
    //   full_cloud.header.frame_id= "tripod_floor_link";
    //   */
    //   pcl_partial_cloud->clear();
    //   pcl_partial_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    //   pcl::fromROSMsg(*cloud_msg,*pcl_partial_cloud);
    // }


   void callback_scan(const mapping_interfaces::msg::ScanTransformedStep::SharedPtr scan_msg) const
    {
      /*
          int scan_step = step_msg->scanstep;
          if (scan_step==1){
            publisher_cloud_->publish(full_cloud);
          }
     */ 
        sensor_msgs::msg::PointCloud2 cloud_msg = scan_msg->cloud;
        pcl_partial_cloud->clear();
        pcl_partial_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(cloud_msg,*pcl_partial_cloud);


        int scan_step = scan_msg->step;

        if(scan_step>0 && scan_step<=8){
          pcl::concatenate(*pcl_full_cloud,*pcl_partial_cloud,*pcl_full_cloud);
        }

        if(scan_step==8){
        sensor_msgs::msg::PointCloud2 full_cloud;
        pcl::toROSMsg(*pcl_full_cloud,full_cloud);
        full_cloud.header.frame_id= "tripod_floor_link";
        publisher_cloud_->publish(full_cloud);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "published");
        pcl_full_cloud->clear();
        pcl_full_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

      }

      std_msgs::msg::Bool finish_s_msg;
      finish_s_msg.data = true;
      publisher_finish_step_->publish(finish_s_msg);
    }

    rclcpp::Subscription<mapping_interfaces::msg::ScanTransformedStep>::SharedPtr subscription_scan_;
    //rclcpp::Subscription<mapping_interfaces::msg::ScanStep>::SharedPtr subscription_step_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_cloud_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_finish_step_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  

  rclcpp::spin(std::make_shared<L360ScanNode>());



  rclcpp::shutdown();



  return 0;
}
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


using std::placeholders::_1;



class ComputeNormal : public rclcpp::Node
{
  public:
    ComputeNormal() : Node("compute_normal")
    {
      subscription_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/scan/full", 10, std::bind(&ComputeNormal::callback_scan, this, std::placeholders::_1));
      publisher_normal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/scan/full_normals",1);
      
    }

  private:
    void callback_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg_scan)
    {
      
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Computing normal");
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*msg_scan, *pclCloud);

      //std::cout << "points: " << pclCloud->size () << std::endl;
      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
      normal_estimation.setInputCloud (pclCloud);
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
      normal_estimation.setSearchMethod (tree);

      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      normal_estimation.setRadiusSearch (0.03);
      normal_estimation.compute (*cloud_normals);

      //std::cout << "cloud_normals->size (): " << cloud_normals->size () << std::endl;

      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::concatenateFields (*pclCloud,*cloud_normals,*outCloud);
      sensor_msgs::msg::PointCloud2 msg_normal;
      pcl::toROSMsg(*outCloud,msg_normal);
      msg_normal.header.frame_id = "tripod_floor_link";
      publisher_normal_->publish(msg_normal);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Completado");

      



      /*
      Eigen::MatrixXf cloud;
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*msg_scan, *pcl_cloud);
      cloud.resize(pcl_cloud->size(),3);

      for (int i = 0; i < pcl_cloud->size(); i++){
        cloud(i,0) = pcl_cloud->points[i].x;
        cloud(i,1) = pcl_cloud->points[i].y;
        cloud(i,2) = pcl_cloud->points[i].z;
      }

      Eigen::MatrixXd covar(cloud.rows(), 3);

      for (int i = 0; i < cloud.rows(); i++){

        std::vector<int> nn_indices;
        std::vector<float> nn_dists;
        pcl::kdtree::kdtree_.nearestKSearch(cloud.row(i), std::k_, nn_indices, nn_dists);


      }
      */


    }

   
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_scan_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_normal_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ComputeNormal>());



  rclcpp::shutdown();



  return 0;
}
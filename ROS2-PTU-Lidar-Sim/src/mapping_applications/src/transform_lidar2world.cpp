#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <utility>
#include <vector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include "tf2_ros/buffer.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <mapping_interfaces/msg/scan_step.hpp>
#include <mapping_interfaces/msg/scan_transformed_step.hpp>

using std::placeholders::_1;
using namespace sensor_msgs::image_encodings;
namespace enc = sensor_msgs::image_encodings;

int count_horizontal=0;
int count_vertical=0;
int state_horizontal=0;
int state_vertical=0;

double fx = 762.7249337622711;
double fy = 762.7249337622711;
double cx = 640.5;
double cy = 360.5;

cv::Mat img_1, img_2, img_3, img_4, img_5, img_6, img_7, img_8;

int state = -1;

int count = 0;
int state_ant = -1;

bool color =  false;
int color_param;
//int state_count = 0;
int count_img = 0;
std::string filename;

//pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_full_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
sensor_msgs::msg::PointCloud2 color_cloud(sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam, sensor_msgs::msg::PointCloud2 transformed_color_cloud_msg, cv::Mat img);


class TransformLidar2world : public rclcpp::Node
{
  public:
    TransformLidar2world() : Node("transform_lidar2world")
    { 
      declare_parameter("color", 1);
      get_parameter("color", color_param);

      if (color_param == 1){
        color = true;
      }

      subscription_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/scan/raw", 0, std::bind(&TransformLidar2world::callback_scan, this, std::placeholders::_1));

      publisher_cloud_step_ = this->create_publisher<mapping_interfaces::msg::ScanTransformedStep>("/lidar/scan/transformed",1);

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      

      //std::cout << "nodo e1 "<< std::endl;
     
    }

  private:
      
    void callback_scan(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {

      //std::cout << "callback" << std::endl;

 
      //  try {
      // // Lookup transform from source frame to target frame
      // geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
      //   "tripod_floor_link",
      //   cloud_msg->header.frame_id,
      //   //cloud_msg->header.stamp,
      //   tf2::TimePointZero
      //   //tf2::durationFromSec(1.0)
      // );

      //  std::cout << "aqui= "<< std::endl;


      // std::cout << "try tripod transform ok" << std::endl;
       // Transform the point cloud using the lookup transform
      // sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
      // tf2::doTransform(*cloud_msg, transformed_cloud_msg, transform_stamped);


      //std::cout << "aqui1.1 "<< std::endl;

      sensor_msgs::msg::PointCloud2 transformed_color_cloud_msg;
      //transformed_color_cloud_msg.header.frame_id = "tripod_floor_link";  
      transformed_color_cloud_msg.header.stamp = cloud_msg->header.stamp;
      
      transformed_color_cloud_msg.height = cloud_msg->height;
      transformed_color_cloud_msg.width = cloud_msg->width;

      transformed_color_cloud_msg.point_step = sizeof(float) * 4;

      transformed_color_cloud_msg.row_step = transformed_color_cloud_msg.width * transformed_color_cloud_msg.point_step;

      //std::cout << "aqui1.2 "<< std::endl;

      sensor_msgs::msg::PointField field_msg_color;
      field_msg_color.name = "x";
      field_msg_color.offset = 0;
      field_msg_color.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_msg_color.count = 1;
      transformed_color_cloud_msg.fields.push_back(field_msg_color);
      //std::cout << "aqui1.3 "<< std::endl;

      field_msg_color.name = "y";
      field_msg_color.offset = sizeof(float);
      field_msg_color.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_msg_color.count = 1;
      transformed_color_cloud_msg.fields.push_back(field_msg_color);
      //std::cout << "aqui1.4 "<< std::endl;

      field_msg_color.name = "z";
      field_msg_color.offset = sizeof(float) * 2;
      field_msg_color.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_msg_color.count = 1;
      transformed_color_cloud_msg.fields.push_back(field_msg_color);
      //std::cout << "aqui1.5 "<< std::endl;

      field_msg_color.name = "rgb";
      field_msg_color.offset = sizeof(float) * 3;
      field_msg_color.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_msg_color.count = 1;
      transformed_color_cloud_msg.fields.push_back(field_msg_color);
      //std::cout << "aqui 2 "<< std::endl;

      transformed_color_cloud_msg.data.resize(transformed_color_cloud_msg.row_step * transformed_color_cloud_msg.height);
     // float* dataPtr = reinterpret_cast<float*>(transformed_color_cloud_msg.data.data());

      // cv::Mat img_result(720,1280, CV_8UC3, cv::Scalar(255,255,255));

      // cv::namedWindow("result", cv::WINDOW_NORMAL );
  	  // cv::imshow("result", img_result);
      // cv::waitKey(1);

      


      //std::cout<<"antes del IF"<<std::endl;
       
      if (state!=state_ant){

        if (color==true){

        //std::cout<<"ENTRO AL IF"<<std::endl;

        switch (state)
        {
          case 1:
            //std::cout<<"case 1"<<std::endl;
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CASE1");
            transformed_color_cloud_msg.header.frame_id = "cam_2";  
              try {
            // Lookup transform from source frame to target frame
            geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
              "cam_2",
              "os_lidar",
              //cloud_msg->header.frame_id,
              //cloud_msg->header.stamp,
              tf2::TimePointZero
              //tf2::durationFromSec(1.0)
            );

            std::cout << "aqui1 "<< std::endl;

              sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
              tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

              //std::cout << "try camera transform ok" << std::endl;

              transformed_color_cloud_msg = color_cloud(transformed_cloud_msg_cam,transformed_color_cloud_msg, img_1);
              //std::cout << "transformed to color "<< std::endl;
              
            } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
            }
            break;
        
          case 2:
            //std::cout<<"case 2"<<std::endl;
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CASE2");
            transformed_color_cloud_msg.header.frame_id = "cam_8";  
              try {
            // Lookup transform from source frame to target frame
            geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
              "cam_8",
              "os_lidar",
              //cloud_msg->header.frame_id,
              //cloud_msg->header.stamp,
              tf2::TimePointZero
              //tf2::durationFromSec(1.0)
            );

            //std::cout << "aqui3 "<< std::endl;

              sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
              tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

              //std::cout << "try camera transform ok" << std::endl;

              transformed_color_cloud_msg = color_cloud( transformed_cloud_msg_cam,transformed_color_cloud_msg, img_2);

            } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
            }
            break;

          case 3:
            //std::cout<<"case 3"<<std::endl;
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CASE3");
            transformed_color_cloud_msg.header.frame_id = "cam_4";  
              try {
            // Lookup transform from source frame to target frame
            geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
              "cam_4",
              "os_lidar",
              //cloud_msg->header.frame_id,
              //cloud_msg->header.stamp,
              tf2::TimePointZero
              //tf2::durationFromSec(1.0)
            );

            //std::cout << "aqui3 "<< std::endl;

              sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
              tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

              //std::cout << "try camera transform ok" << std::endl;

              transformed_color_cloud_msg = color_cloud( transformed_cloud_msg_cam,transformed_color_cloud_msg, img_3);

            } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
            }
            break;

          case 4:
            //std::cout<<"case 4"<<std::endl;
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CASE4");
            transformed_color_cloud_msg.header.frame_id = "cam_6";  
              try {
            // Lookup transform from source frame to target frame
            geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
              "cam_6",
              "os_lidar",
              //cloud_msg->header.frame_id,
              //cloud_msg->header.stamp,
              tf2::TimePointZero
              //tf2::durationFromSec(1.0)
            );

            //std::cout << "aqui3 "<< std::endl;

              sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
              tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

              //std::cout << "try camera transform ok" << std::endl;

              transformed_color_cloud_msg = color_cloud( transformed_cloud_msg_cam,transformed_color_cloud_msg, img_4);

            } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
            }
            break;

          case 5:
            //std::cout<<"case 5"<<std::endl;
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CASE5");
            transformed_color_cloud_msg.header.frame_id = "cam_1";  
              try {
            // Lookup transform from source frame to target frame
            geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
              "cam_1",
              "os_lidar",
              //cloud_msg->header.frame_id,
              //cloud_msg->header.stamp,
              tf2::TimePointZero
              //tf2::durationFromSec(1.0)
            );

            //std::cout << "aqui3 "<< std::endl;

              sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
              tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

              //std::cout << "try camera transform ok" << std::endl;

              transformed_color_cloud_msg = color_cloud( transformed_cloud_msg_cam,transformed_color_cloud_msg, img_5);

            } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
            }
            break;

          case 6:
            //std::cout<<"case 6"<<std::endl;
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CASE6");
            transformed_color_cloud_msg.header.frame_id = "cam_5";  
              try {
            // Lookup transform from source frame to target frame
            geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
              "cam_5",
              "os_lidar",
              //cloud_msg->header.frame_id,
              //cloud_msg->header.stamp,
              tf2::TimePointZero
              //tf2::durationFromSec(1.0)
            );

            //std::cout << "aqui3 "<< std::endl;

              sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
              tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

              //std::cout << "try camera transform ok" << std::endl;

              transformed_color_cloud_msg = color_cloud( transformed_cloud_msg_cam,transformed_color_cloud_msg, img_6);

            } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
            }
            break;

          case 7:
            //std::cout<<"case 7"<<std::endl;
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CASE7");
            transformed_color_cloud_msg.header.frame_id = "cam_3";  
              try {
            // Lookup transform from source frame to target frame
            geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
              "cam_3",
              "os_lidar",
              //cloud_msg->header.frame_id,
              //cloud_msg->header.stamp,
              tf2::TimePointZero
              //tf2::durationFromSec(1.0)
            );

            //std::cout << "aqui3 "<< std::endl;

              sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
              tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

              //std::cout << "try camera transform ok" << std::endl;

              transformed_color_cloud_msg = color_cloud( transformed_cloud_msg_cam,transformed_color_cloud_msg, img_7);

            } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
            }
            break;

          case 8:
            //std::cout<<"case 8"<<std::endl;
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CASE8");
            transformed_color_cloud_msg.header.frame_id = "cam_7";  
              try {
            // Lookup transform from source frame to target frame
            geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
              "cam_7",
              "os_lidar",
              //cloud_msg->header.frame_id,
              //cloud_msg->header.stamp,
              tf2::TimePointZero
              //tf2::durationFromSec(1.0)
            );

            //std::cout << "aqui3 "<< std::endl;

              sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
              tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

              //std::cout << "try camera transform ok" << std::endl;

              transformed_color_cloud_msg = color_cloud( transformed_cloud_msg_cam,transformed_color_cloud_msg, img_8);

            } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
            }
            break;
      
          default:
            break;
        } 
          //std::cout << "Finish cases "<< std::endl;

      /*  std::cout << "aqui 10"<< std::endl;

        cv::cvtColor(img_result,img_result,cv::COLOR_BGR2RGB);
        std::cout << "aqui 10.1"<< std::endl;
        cv::namedWindow("result2", cv::WINDOW_NORMAL );
        std::cout << "aqui 10.2"<< std::endl;
        cv::imshow("result2", img_result);
        std::cout << "aqui 10.3"<< std::endl;
        cv::waitKey(1);
        std::cout << "aqui 10.4"<< std::endl;


        std::cout << "aqui 11"<< std::endl; */


        try {
          // Lookup transform from source frame to target frame
          geometry_msgs::msg::TransformStamped transform_stamped_floor = tf_buffer_->lookupTransform(
            "tripod_floor_link",
            //transformed_color_cloud_msg.header.frame_id,
            //cloud_msg->header.frame_id,
            "os_lidar",
            //"cam_1",
            //cloud_msg->header.stamp,
            tf2::TimePointZero
            //tf2::durationFromSec(1.0)
          );
          //std::cout << "try transform "<< std::endl;
      // std::cout << "aqui 12"<< std::endl;
            
            sensor_msgs::msg::PointCloud2 transformed_cloud_msg;

            //std::cout << "aqui 13"<< std::endl;
            //tf2::doTransform(transformed_color_cloud_msg, transformed_cloud_msg, transform_stamped_floor);
            tf2::doTransform(*cloud_msg, transformed_color_cloud_msg, transform_stamped_floor);
            
          //std::cout << "do transform "<< std::endl;
            //std::cout << "aqui 14"<< std::endl;

            // if (state_ant == state_count){
            //   count ++;
            //   std::cout << "aqui a"<<  count << std::endl;
            // }

            // else {
            //   count = 0;
            //   std::cout << "aqui b++++++++++++++++++++++"<< std::endl;
            // }


            // if (count == 2){
              //publisher_cloud_->publish(transformed_cloud_msg);
              mapping_interfaces::msg::ScanTransformedStep msg;
              msg.cloud = transformed_color_cloud_msg ;
              msg.step = state;
              publisher_cloud_step_->publish(msg);
              std::cout << " Publicado" << count << std::endl;
            //   state_count = 0;
            // }
            

            // state_ant = state_count;
            


          } catch (tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
          }
        
        
      }

      
      else{
        try {
          // Lookup transform from source frame to target frame
          geometry_msgs::msg::TransformStamped transform_stamped_floor = tf_buffer_->lookupTransform(
            "tripod_floor_link",
            "os_lidar",
            tf2::TimePointZero
          );
          //std::cout << "try transform "<< std::endl;
          sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
          tf2::doTransform(*cloud_msg, transformed_color_cloud_msg, transform_stamped_floor);
            
          //std::cout << "do transform "<< std::endl;

              mapping_interfaces::msg::ScanTransformedStep msg;
              msg.cloud = transformed_color_cloud_msg ;
              msg.step = state;
              publisher_cloud_step_->publish(msg);
              std::cout << " Publicado" << count << std::endl;

        

          } catch (tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
          }
      }
      
      state_ant = state;
    }
//std::cout << "aqui 15"<< std::endl;
      //std::cout << " Antes de publicar " << std::endl;

      // Publish the transformed point cloud
      
 
      // } catch (tf2::TransformException &ex) {
      //   RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
      //   } 
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_scan_;
    rclcpp::Publisher<mapping_interfaces::msg::ScanTransformedStep>::SharedPtr publisher_cloud_step_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

class SubsNodeImg : public rclcpp::Node
{
  public:
    SubsNodeImg() : Node("subs_node_img")
    {
      subscription_cam_1 = this->create_subscription<sensor_msgs::msg::Image>(
      "cam_1", 10, std::bind(&SubsNodeImg::cam_1_callback, this, std::placeholders::_1));

      subscription_cam_2 = this->create_subscription<sensor_msgs::msg::Image>(
      "cam_2", 10, std::bind(&SubsNodeImg::cam_2_callback, this, std::placeholders::_1));

      subscription_cam_3 = this->create_subscription<sensor_msgs::msg::Image>(
      "cam_3", 10, std::bind(&SubsNodeImg::cam_3_callback, this, std::placeholders::_1));

      subscription_cam_4 = this->create_subscription<sensor_msgs::msg::Image>(
      "cam_4", 10, std::bind(&SubsNodeImg::cam_4_callback, this, std::placeholders::_1));

      subscription_cam_5 = this->create_subscription<sensor_msgs::msg::Image>(
      "cam_5", 10, std::bind(&SubsNodeImg::cam_5_callback, this, std::placeholders::_1));

      subscription_cam_6 = this->create_subscription<sensor_msgs::msg::Image>(
      "cam_6", 10, std::bind(&SubsNodeImg::cam_6_callback, this, std::placeholders::_1));

      subscription_cam_7 = this->create_subscription<sensor_msgs::msg::Image>(
      "cam_7", 10, std::bind(&SubsNodeImg::cam_7_callback, this, std::placeholders::_1));

      subscription_cam_8 = this->create_subscription<sensor_msgs::msg::Image>(
      "cam_8", 10, std::bind(&SubsNodeImg::cam_8_callback, this, std::placeholders::_1));

      std::cout << "config node img" << std::endl;
    }

  private:

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_cam_1;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_cam_2;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_cam_3;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_cam_4;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_cam_5;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_cam_6;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_cam_7;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_cam_8;


    void cam_1_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_1 = image_bridge->image;
      cv::cvtColor(img_1,img_1,cv::COLOR_BGR2RGB);
      //std::cout << "Update cam img" << std::endl;
      //cv::namedWindow("img_1", cv::WINDOW_NORMAL );
  	  //cv::imshow("img_1", img_1);
  	  //cv::waitKey(1);
      //std::cout << "callback1" << std::endl;
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CallbackImage1");

    }

    void cam_2_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_2 = image_bridge->image;
      cv::cvtColor(img_2,img_2,cv::COLOR_BGR2RGB);
      // cv::namedWindow("img_2", cv::WINDOW_NORMAL );  std::cout << "nodo e1 "<< std::endl;
  	  // cv::imshow("img_2", img_2);
  	  // cv::waitKey(1);
      //std::cout << "callback2" << std::endl;
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CallbackImage2");
    }

    void cam_3_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_3 = image_bridge->image;
      cv::cvtColor(img_3,img_3,cv::COLOR_BGR2RGB);
      // cv::namedWindow("img_3", cv::WINDOW_NORMAL );
  	  // cv::imshow("img_3", img_3);
  	  // cv::waitKey(1);
      //std::cout << "callback3" << std::endl;
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CallbackImage3");
    }

    void cam_4_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_4 = image_bridge->image;
      cv::cvtColor(img_4,img_4,cv::COLOR_BGR2RGB);
      // cv::namedWindow("img_4", cv::WINDOW_NORMAL );
  	  // cv::imshow("img_4", img_4);
  	  // cv::waitKey(1);
      //std::cout << "callback4" << std::endl;
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CallbackImage4");
    }

    void cam_5_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_5 = image_bridge->image;
      cv::cvtColor(img_5,img_5,cv::COLOR_BGR2RGB);
      // cv::namedWindow("img_5", cv::WINDOW_NORMAL );
  	  // cv::imshow("img_5", img_5);
  	  // cv::waitKey(1);
      //std::cout << "callback5" << std::endl;
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CallbackImage5");
    }

    void cam_6_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_6 = image_bridge->image;
      cv::cvtColor(img_6,img_6,cv::COLOR_BGR2RGB);
      // cv::namedWindow("img_6", cv::WINDOW_NORMAL );
  	  // cv::imshow("img_6", img_6);
  	  // cv::waitKey(1);
      //std::cout << "callback6" << std::endl;
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CallbackImage6");
    }

    void cam_7_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_7 = image_bridge->image;
      cv::cvtColor(img_7,img_7,cv::COLOR_BGR2RGB);
      // cv::namedWindow("img_7", cv::WINDOW_NORMAL );
  	  // cv::imshow("img_7", img_7);
  	  // cv::waitKey(1);
      //std::cout << "callback7" << std::endl;
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CallbackImage7");
    }

    void cam_8_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_8 = image_bridge->image;
      cv::cvtColor(img_8,img_8,cv::COLOR_BGR2RGB);
      // cv::namedWindow("img_8", cv::WINDOW_NORMAL );
  	  // cv::imshow("img_8", img_8);
  	  // cv::waitKey(1);
      //std::cout << "callback8" << std::endl;
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CallbackImage8");
    }

};



class SubsNodeL2w : public rclcpp::Node
{
  public:
    SubsNodeL2w() : Node("subs_node_l2w")
    {
      

      subscription_step_ = this->create_subscription<mapping_interfaces::msg::ScanStep>(
      "/step_scan", 10, std::bind(&SubsNodeL2w::callback_step, this, std::placeholders::_1));


      //std::cout << "config node l2w" << std::endl;
    }

  private:

    
    rclcpp::Subscription<mapping_interfaces::msg::ScanStep>::SharedPtr subscription_step_;

    void callback_step(const mapping_interfaces::msg::ScanStep::SharedPtr step_msg) const
    {
      state = step_msg->scanstep;

      //std::cout << "state " << state << std::endl;

    }
    
};



sensor_msgs::msg::PointCloud2 color_cloud(sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam, sensor_msgs::msg::PointCloud2 transformed_color_cloud_msg, cv::Mat img){
        
        //std::cout << "inicio color"<< std::endl;
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inicio Color");
        float* dataPtr = reinterpret_cast<float*>(transformed_color_cloud_msg.data.data());
        //std::cout << "pointer para pc2"<< std::endl;
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pointer PC2");
        const uint32_t numPoints = transformed_cloud_msg_cam.width * transformed_cloud_msg_cam.height;
        const uint32_t pointStep = transformed_cloud_msg_cam.point_step;
        const auto& fields = transformed_cloud_msg_cam.fields;
        const uint8_t* data = transformed_cloud_msg_cam.data.data();

        int j=0;
        //std::cout << "declara variables"<< std::endl;
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Declara Variables");

        for (uint32_t i = 0; i < numPoints; ++i)
            {   
              //std::cout << "primer for"<< std::endl;
              //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Primer for");
                float x, y, z;
                const uint32_t offset = i * pointStep;

               // std::cout << "offset " << offset << " / " << std::endl;

                // Interpret x, y, z fields based on their offsets and data type
                for (const auto& field : fields) 
                {
                  //std::cout << "segundo for"<< std::endl;
                  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Segundo for");
                    const uint32_t fieldOffset = offset + field.offset;
                    const uint8_t* fieldData = data + fieldOffset;

                    if (field.name == "x")
                    {
                      x = *reinterpret_cast<const float*>(fieldData);  
                      //std::cout << "x " << x << " / " << std::endl;  
                    }
                    else if (field.name == "y")
                    {
                      y = *reinterpret_cast<const float*>(fieldData);
                      //std::cout << "y " << y << " / " << std::endl;  
                    }
                    else if (field.name == "z")
                    {
                      z = *reinterpret_cast<const float*>(fieldData);
                      //std::cout << "z " << z << " / " << std::endl;   
                    }
 
                }

             //std::cout << "termina 2 for "<< std::endl;
             //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Termina for 2");

              dataPtr[j] = x;    // x of point 1
              dataPtr[j+1] = y;    // y of point 1
              dataPtr[j+2] = z;    // z of point 1
              //dataPtr[j+3] = 0.5;    // intensity of point 1

             //std::cout << "termina asignacion de valores en dataPtr"<< std::endl;
             //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Termina asignacion de valores en dataPtr");
             
              //double pointx =  (x * fx) / z;
              //double pointy =  (y * fy) / z;
              //int pointx = int((((y * fx) / x) - (1280/2))*-1);
              //int pointy = (int((((z * fy) / x) - (720/2))*-1));

              std::cout << "XC= " << x << "  YC= "  << y << "  ZC= "  << z << std::endl;


              int pointx = int((((x * fx) / z) - (1280.0/2.0))*1.0);
              int pointy = (int((((y * fy) / z) - (720.0/2.0))));
              
              //std::cout << "busca punto x y y"<< std::endl;
              //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Busca punto X y Y");


              int blue = 0;

              int green = 0;

              int red = 0;

            //  std::cout << "X= " << x << "  Y= "  << y << "  Z= " << z << std::endl;

              std::cout << "PointX= " << pointx << "  PointY= "  << pointy << std::endl;
              //std::cout << "Image shape = " << img.cols << "   Image y size = " << img.rows << std::endl;

              if (pointx >= 0 && pointx <= 1280 && pointy >=0 && pointy <= 720){
                //std::cout << "if para asignar puntos x y en camara"<< std::endl;
                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "If para asignar puntos X Y en camara");
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Si hay color, entrada");
                
                //cv::flip(img_1,img_1,1);
                
                //count_img++;
                //filename = "image_" + std::to_string(count_img) + ".jpg"; 
                //cv::imwrite(filename,img);
                cv::Vec3b RGB_color = img.at<cv::Vec3b>(pointy,(pointx));

                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "copy vector color");

                blue = RGB_color[2];

                green = RGB_color[1];

                red = RGB_color[0];

               // img_result.at<cv::Vec3b>(pointy,pointx)= RGB_color;


            //    std::cout << "R= " << red << "  G= "  << green << "  B=" << blue << std::endl;

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Si hay color, salida");
                

              }

              else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No hay color");
              }


            //  std::cout << "aqui 7"<< std::endl;

              
              //std::cout << "antes de declara variables de color"<< std::endl;
              //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Antes de declarar variables de color");
              int rgbValue = 0;
              rgbValue |= static_cast<uint32_t>(red) << 16;    // Red component
              rgbValue |= static_cast<uint32_t>(green) << 8;   // Green component
              rgbValue |= static_cast<uint32_t>(blue);         // Blue component

              float float_rgb = *reinterpret_cast<float*>(&rgbValue); 
              
              //std::cout << "antes de asignar color a PTR"<< std::endl;
              //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Antes de asignar color a PTR");
              dataPtr[j+3] = float_rgb;                         // RGB color
              //std::cout << "despues de asignar color"<< std::endl;
              //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Despues de asignar color");
              //std::cout << rgbValue << std::endl;
            

            //  std::cout << "aqui 9"<< std::endl;

              j = j + 4; 

          }
          //std::cout << "antes del return"<< std::endl;
          //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Antes del return");
          return transformed_color_cloud_msg;
}




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  //rclcpp::spin(std::make_shared<TransformLidar2world>());


  auto node = std::make_shared<TransformLidar2world>();
  auto subs_node_l2w = std::make_shared<SubsNodeL2w>();
  auto subs_node_img = std::make_shared<SubsNodeImg>();

  

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(subs_node_l2w);
  executor.add_node(subs_node_img);
  executor.spin(); 



  rclcpp::shutdown();



  return 0;
}
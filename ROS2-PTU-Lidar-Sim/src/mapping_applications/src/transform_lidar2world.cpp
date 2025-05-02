/**
Copyright (C) 2025 Intel Corporation
SPDX-License-Identifier: Apache-2.0
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
 +File name: transform_lidar2world.cpp
 +Description: Transforms lidar point cloud data to the world frame and applies optional color mapping.
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

// Global variables for image dimensions and camera intrinsic parameters
double image_width = 1280.0; // Width of the image
double image_height = 1440.0; // Height of the image
double fx = 762.72499084472656;  // Focal length in x-direction
double fy = 762.72497177124023;  // Focal length in y-direction
double cx = image_width / 2.0;   // Principal point x-coordinate
double cy = image_height / 2.0;  // Principal point y-coordinate

cv::Mat img_1, img_2, img_3, img_4, img_5, img_6, img_7, img_8;

int state = -1;

int count = 0;
int state_ant = -1;

bool color =  false;
int color_param;
//int state_count = 0;
int count_img = 0;
std::string filename;

sensor_msgs::msg::PointCloud2 color_cloud(sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam, sensor_msgs::msg::PointCloud2 transformed_color_cloud_msg, cv::Mat img);

class TransformLidar2world : public rclcpp::Node
{
  public:
    TransformLidar2world() : Node("transform_lidar2world")
    { 
      // Declare and retrieve the "color" parameter to enable or disable color mapping
      declare_parameter("color", 1);
      get_parameter("color", color_param);

      if (color_param == 1){
        color = true;
      }

      // Subscribe to raw lidar scan topic
      subscription_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/scan/raw", 0, std::bind(&TransformLidar2world::callback_scan, this, std::placeholders::_1));

      // Publishers for transformed lidar data
      publisher_cloud_step_ = this->create_publisher<mapping_interfaces::msg::ScanTransformedStep>("/lidar/scan/transformed",1);
      publisher_prev_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/scan/prev",1);

      // Initialize TF2 buffer and listener for coordinate frame transformations
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

  private:
    void callback_scan(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
      // Callback function to process incoming lidar point cloud data

      sensor_msgs::msg::PointCloud2 transformed_color_cloud_msg;
      transformed_color_cloud_msg.header.stamp = cloud_msg->header.stamp;
      
      transformed_color_cloud_msg.height = cloud_msg->height;
      transformed_color_cloud_msg.width = cloud_msg->width;

      transformed_color_cloud_msg.point_step = sizeof(float) * 4;

      transformed_color_cloud_msg.row_step = transformed_color_cloud_msg.width * transformed_color_cloud_msg.point_step;

      sensor_msgs::msg::PointField field_msg_color;
      field_msg_color.name = "x";
      field_msg_color.offset = 0;
      field_msg_color.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_msg_color.count = 1;
      transformed_color_cloud_msg.fields.push_back(field_msg_color);

      field_msg_color.name = "y";
      field_msg_color.offset = sizeof(float);
      field_msg_color.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_msg_color.count = 1;
      transformed_color_cloud_msg.fields.push_back(field_msg_color);

      field_msg_color.name = "z";
      field_msg_color.offset = sizeof(float) * 2;
      field_msg_color.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_msg_color.count = 1;
      transformed_color_cloud_msg.fields.push_back(field_msg_color);

      field_msg_color.name = "rgb";
      field_msg_color.offset = sizeof(float) * 3;
      field_msg_color.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_msg_color.count = 1;
      transformed_color_cloud_msg.fields.push_back(field_msg_color);

      transformed_color_cloud_msg.data.resize(transformed_color_cloud_msg.row_step * transformed_color_cloud_msg.height);

      if (state != state_ant) {
        if (color == true) {
          // Switch case to handle different camera states and apply transformations
          switch (state) {
            case 1:
              transformed_color_cloud_msg.header.frame_id = "cam_2";  
              try {
                // Lookup transform from lidar to camera frame
                geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
                  "cam_2", "os_lidar", tf2::TimePointZero);

                // Transform the point cloud to the camera frame
                sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
                tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

                // Apply color mapping using the camera image
                transformed_color_cloud_msg = color_cloud(transformed_cloud_msg_cam, transformed_color_cloud_msg, img_2);

                // Transform the colored point cloud back to the lidar frame
                try {
                  geometry_msgs::msg::TransformStamped tranform_stamped_lidar = tf_buffer_->lookupTransform(
                    "os_lidar", "cam_2", tf2::TimePointZero);
                  tf2::doTransform(transformed_color_cloud_msg, transformed_color_cloud_msg, tranform_stamped_lidar);
                } catch (tf2::TransformException &ex) {
                  RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
                }
              } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
              }
              break;

            case 2:
              transformed_color_cloud_msg.header.frame_id = "cam_8";  
              try {
                geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
                  "cam_8", "os_lidar", tf2::TimePointZero);

                sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
                tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

                transformed_color_cloud_msg = color_cloud(transformed_cloud_msg_cam, transformed_color_cloud_msg, img_8);

                try {
                  geometry_msgs::msg::TransformStamped tranform_stamped_lidar = tf_buffer_->lookupTransform(
                    "os_lidar", "cam_8", tf2::TimePointZero);
                  tf2::doTransform(transformed_color_cloud_msg, transformed_color_cloud_msg, tranform_stamped_lidar);
                } catch (tf2::TransformException &ex) {
                  RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
                }
              } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
              }
              break;

            case 3:
              transformed_color_cloud_msg.header.frame_id = "cam_4";
              try {
                geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
                  "cam_4", "os_lidar", tf2::TimePointZero);

                sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
                tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

                transformed_color_cloud_msg = color_cloud(transformed_cloud_msg_cam, transformed_color_cloud_msg, img_4);

                try {
                  geometry_msgs::msg::TransformStamped tranform_stamped_lidar = tf_buffer_->lookupTransform(
                    "os_lidar", "cam_4", tf2::TimePointZero);
                  tf2::doTransform(transformed_color_cloud_msg, transformed_color_cloud_msg, tranform_stamped_lidar);
                } catch (tf2::TransformException &ex) {
                  RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
                }
              } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
              }
              break;

            case 4:
              transformed_color_cloud_msg.header.frame_id = "cam_6";  
              try {
                geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
                  "cam_6", "os_lidar", tf2::TimePointZero);

                sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
                tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

                transformed_color_cloud_msg = color_cloud(transformed_cloud_msg_cam, transformed_color_cloud_msg, img_6);

                try {
                  geometry_msgs::msg::TransformStamped tranform_stamped_lidar = tf_buffer_->lookupTransform(
                    "os_lidar", "cam_6", tf2::TimePointZero);
                  tf2::doTransform(transformed_color_cloud_msg, transformed_color_cloud_msg, tranform_stamped_lidar);
                } catch (tf2::TransformException &ex) {
                  RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
                }
              } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
              }
              break;

            case 5:
              transformed_color_cloud_msg.header.frame_id = "cam_1";  
              try {
                geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
                  "cam_1", "os_lidar", tf2::TimePointZero);

                sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
                tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

                transformed_color_cloud_msg = color_cloud(transformed_cloud_msg_cam, transformed_color_cloud_msg, img_1);

                try {
                  geometry_msgs::msg::TransformStamped tranform_stamped_lidar = tf_buffer_->lookupTransform(
                    "os_lidar", "cam_1", tf2::TimePointZero);
                  tf2::doTransform(transformed_color_cloud_msg, transformed_color_cloud_msg, tranform_stamped_lidar);
                } catch (tf2::TransformException &ex) {
                  RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
                }
              } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
              }
              break;

            case 6:
              transformed_color_cloud_msg.header.frame_id = "cam_5";  
              try {
                geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
                  "cam_5", "os_lidar", tf2::TimePointZero);

                sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
                tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

                transformed_color_cloud_msg = color_cloud(transformed_cloud_msg_cam, transformed_color_cloud_msg, img_5);

                try {
                  geometry_msgs::msg::TransformStamped tranform_stamped_lidar = tf_buffer_->lookupTransform(
                    "os_lidar", "cam_5", tf2::TimePointZero);
                  tf2::doTransform(transformed_color_cloud_msg, transformed_color_cloud_msg, tranform_stamped_lidar);
                } catch (tf2::TransformException &ex) {
                  RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
                }
              } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
              }
              break;

            case 7:
              transformed_color_cloud_msg.header.frame_id = "cam_3";
              try {
                geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
                  "cam_3", "os_lidar", tf2::TimePointZero);

                sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
                tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

                transformed_color_cloud_msg = color_cloud(transformed_cloud_msg_cam, transformed_color_cloud_msg, img_3);

                try {
                  geometry_msgs::msg::TransformStamped tranform_stamped_lidar = tf_buffer_->lookupTransform(
                    "os_lidar", "cam_3", tf2::TimePointZero);
                  tf2::doTransform(transformed_color_cloud_msg, transformed_color_cloud_msg, tranform_stamped_lidar);
                } catch (tf2::TransformException &ex) {
                  RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
                }
              } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
              }
              break;

            case 8:
              transformed_color_cloud_msg.header.frame_id = "cam_7";
              try {
                geometry_msgs::msg::TransformStamped transform_stamped_cam = tf_buffer_->lookupTransform(
                  "cam_7", "os_lidar", tf2::TimePointZero);

                sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam;
                tf2::doTransform(*cloud_msg, transformed_cloud_msg_cam, transform_stamped_cam);

                transformed_color_cloud_msg = color_cloud(transformed_cloud_msg_cam, transformed_color_cloud_msg, img_7);

                try {
                  geometry_msgs::msg::TransformStamped tranform_stamped_lidar = tf_buffer_->lookupTransform(
                    "os_lidar", "cam_7", tf2::TimePointZero);
                  tf2::doTransform(transformed_color_cloud_msg, transformed_color_cloud_msg, tranform_stamped_lidar);
                } catch (tf2::TransformException &ex) {
                  RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
                }
              } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
              }
              break;

            default:
              break;
          }

          try {
            // Transform the colored point cloud to the world frame
            geometry_msgs::msg::TransformStamped transform_stamped_floor = tf_buffer_->lookupTransform(
              "tripod_floor_link", "os_lidar", tf2::TimePointZero);
            sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
            tf2::doTransform(transformed_color_cloud_msg, transformed_cloud_msg, transform_stamped_floor);

            // Publish the transformed point cloud and step information
            mapping_interfaces::msg::ScanTransformedStep msg;
            msg.cloud = transformed_cloud_msg;
            msg.step = state;
            publisher_cloud_step_->publish(msg);

            // Publish the previous point cloud
            sensor_msgs::msg::PointCloud2 transformed_prev_cloud_msg = transformed_cloud_msg;
            publisher_prev_cloud_->publish(transformed_prev_cloud_msg);
          } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
          }
        }
      } else {
        // Handle case when state remains unchanged
        try {
          geometry_msgs::msg::TransformStamped transform_stamped_floor = tf_buffer_->lookupTransform(
            "tripod_floor_link", "os_lidar", tf2::TimePointZero);
          sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
          tf2::doTransform(*cloud_msg, transformed_color_cloud_msg, transform_stamped_floor);

          mapping_interfaces::msg::ScanTransformedStep msg;
          msg.cloud = transformed_color_cloud_msg;
          msg.step = state;
          publisher_cloud_step_->publish(msg);
        } catch (tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
        }
      }

      state_ant = state; // Update the previous state
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_scan_;
    rclcpp::Publisher<mapping_interfaces::msg::ScanTransformedStep>::SharedPtr publisher_cloud_step_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_prev_cloud_;
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
    }

    void cam_2_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_2 = image_bridge->image;
      cv::cvtColor(img_2,img_2,cv::COLOR_BGR2RGB);
    }

    void cam_3_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_3 = image_bridge->image;
      cv::cvtColor(img_3,img_3,cv::COLOR_BGR2RGB);
    }

    void cam_4_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_4 = image_bridge->image;
      cv::cvtColor(img_4,img_4,cv::COLOR_BGR2RGB);
    }

    void cam_5_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_5 = image_bridge->image;
      cv::cvtColor(img_5,img_5,cv::COLOR_BGR2RGB);
    }

    void cam_6_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_6 = image_bridge->image;
      cv::cvtColor(img_6,img_6,cv::COLOR_BGR2RGB);
    }

    void cam_7_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_7 = image_bridge->image;
      cv::cvtColor(img_7,img_7,cv::COLOR_BGR2RGB);
    }

    void cam_8_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImageConstPtr image_bridge;
      image_bridge=cv_bridge::toCvCopy(msg);
      img_8 = image_bridge->image;
      cv::cvtColor(img_8,img_8,cv::COLOR_BGR2RGB);
    }
};

class SubsNodeL2w : public rclcpp::Node
{
  public:
    SubsNodeL2w() : Node("subs_node_l2w")
    {
      subscription_step_ = this->create_subscription<mapping_interfaces::msg::ScanStep>(
      "/step_scan", 10, std::bind(&SubsNodeL2w::callback_step, this, std::placeholders::_1));
    }

  private:
    rclcpp::Subscription<mapping_interfaces::msg::ScanStep>::SharedPtr subscription_step_;

    void callback_step(const mapping_interfaces::msg::ScanStep::SharedPtr step_msg) const
    {
      state = step_msg->scanstep;
    }
};

sensor_msgs::msg::PointCloud2 color_cloud(sensor_msgs::msg::PointCloud2 transformed_cloud_msg_cam, sensor_msgs::msg::PointCloud2 transformed_color_cloud_msg, cv::Mat img) {
  // Function to apply color mapping to the point cloud using the camera image

  float* dataPtr = reinterpret_cast<float*>(transformed_color_cloud_msg.data.data());
  const uint32_t numPoints = transformed_cloud_msg_cam.width * transformed_cloud_msg_cam.height;
  const uint32_t pointStep = transformed_cloud_msg_cam.point_step;
  const auto& fields = transformed_cloud_msg_cam.fields;
  const uint8_t* data = transformed_cloud_msg_cam.data.data();

  int j = 0;

  // For cycle to fill the rgb field with 0
  for (uint32_t i = 0; i < numPoints; ++i) {
    dataPtr[j + 3] = 0;
    j = j + 4;
  }

  // Restart j
  j = 0;

  for (uint32_t i = 0; i < numPoints; ++i) {   
    float x, y, z;
    const uint32_t offset = i * pointStep;

    // Interpret x, y, z fields based on their offsets and data type
    for (const auto& field : fields) {
      const uint32_t fieldOffset = offset + field.offset;
      const uint8_t* fieldData = data + fieldOffset;

      if (field.name == "x") {
        x = *reinterpret_cast<const float*>(fieldData);  
      } else if (field.name == "y") {
        y = *reinterpret_cast<const float*>(fieldData);
      } else if (field.name == "z") {
        z = *reinterpret_cast<const float*>(fieldData);
      }
    }

    dataPtr[j] = x;    // x of point 1
    dataPtr[j + 1] = y;    // y of point 1
    dataPtr[j + 2] = z;    // z of point 1

    int pointx = int((((y * fx) / x) - (image_width / 2.0)) * -1.0);
    int pointy = (int((((z * fy) / x) - (image_height / 2.0))) * -1.0);

    int blue = 0;
    int green = 0;
    int red = 0;

    std::cout << "PointX= " << pointx << "  PointY= "  << pointy << std::endl;

    if (pointx >= 0 && pointx <= image_width && pointy >= 0 && pointy <= image_height) {
      cv::Vec3b RGB_color = img.at<cv::Vec3b>(pointy, (pointx));

      blue = RGB_color[0];
      green = RGB_color[1];
      red = RGB_color[2];

      int rgbValue = 0;
      rgbValue |= static_cast<uint32_t>(red) << 16;    // Red component
      rgbValue |= static_cast<uint32_t>(green) << 8;   // Green component
      rgbValue |= static_cast<uint32_t>(blue);         // Blue component

      float float_rgb = *reinterpret_cast<float*>(&rgbValue); 
      dataPtr[j + 3] = float_rgb;                         // RGB color
    }

    j = j + 4; 
  }

  return transformed_color_cloud_msg;
}

int main(int argc, char * argv[])
{
  // Main function to initialize ROS2 nodes and spin the executor
  rclcpp::init(argc, argv);

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
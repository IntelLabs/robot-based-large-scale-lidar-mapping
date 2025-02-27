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
#include <fstream>


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>


using std::placeholders::_1;
int x_file=1;
int param_normals_;
int save_ply_;

float pan_angle, tilt_angle;
geometry_msgs::msg::Pose model_pose;


class PlyConverter : public rclcpp::Node
{
  public:
    PlyConverter() : Node("ply_converter")
    {

      declare_parameter("param_normals", 0);
      get_parameter("param_normals", param_normals_);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "param_normals %d", param_normals_);

      declare_parameter("save_ply", 0);
      get_parameter("save_ply", save_ply_);

      if (param_normals_== 1){
        subscription_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar/scan/full_normals", 10, std::bind(&PlyConverter::callback_scan, this, std::placeholders::_1));
      }
      else{
        subscription_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar/scan/full", 10, std::bind(&PlyConverter::callback_scan, this, std::placeholders::_1));
      }

      //  subscription_pan_tilt_ = this->create_subscription<interfaces::msg::PosLidar>(
      //   "/pan_tilt_angle", 10, std::bind(&PlyConverter::callback_pan_tilt, this, std::placeholders::_1));

      subscription_pan_angle_ = this->create_subscription<std_msgs::msg::Float64>(
        "/pan_pos_controller", 10, std::bind(&PlyConverter::callback_pan_angle, this, std::placeholders::_1));
      
      subscription_tilt_angle_ = this->create_subscription<std_msgs::msg::Float64>(
        "/tilt_pos_controller", 10, std::bind(&PlyConverter::callback_tilt_angle, this, std::placeholders::_1));

        subscription_model_pose_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/set_ptu_position", 10, std::bind(&PlyConverter::callback_model_pose, this, std::placeholders::_1));

      
      
    }

  private:
    void callback_pan_angle(std_msgs::msg::Float64::SharedPtr msg_pan)
    {
      pan_angle = msg_pan->data;
    }

    void callback_tilt_angle(std_msgs::msg::Float64::SharedPtr msg_tilt)
    {
      tilt_angle = msg_tilt->data;
    }


    void callback_model_pose(const geometry_msgs::msg::Pose::SharedPtr msg_model_pose)
    {
      model_pose.position.x = msg_model_pose->position.x;
      model_pose.position.y = msg_model_pose->position.y;
      model_pose.position.z = msg_model_pose->position.z;
      model_pose.orientation.x = msg_model_pose->orientation.x;
      model_pose.orientation.y = msg_model_pose->orientation.y;
      model_pose.orientation.z = msg_model_pose->orientation.z;
      model_pose.orientation.w = msg_model_pose->orientation.w;

    }
    void callback_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg_scan)
    {
      if (save_ply_==1){

        


  /// LIBRARY PCL PLY
        /*//pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud;
        pcl::PointCloud<pcl::PointXYZI> pclCloud;
        pcl::fromROSMsg(*msg_scan, pclCloud);
        pcl::PLYWriter writer;
        bool binary = false;
        bool use_camera = false;
        //std::stringstream filePath='home'.srt();

        std::ostringstream path;
        path << "ply/scan_" << x << ".ply";
        std::string filePath = path.str();
        x++;

        if (writer.write(filePath, pclCloud, binary, use_camera) != 0) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR");
        }
        else{
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Completado");
        }
        */
  /////////////////////////////////////////////////


        std::ostringstream path;
        if (param_normals_==1){
          path << "ply_normals/scan_" << x_file << ".ply";
        }
        else{
          path << "ply/scan_" << x_file << ".ply";
        }

        std::string filePath = path.str();
        x_file++;


  /// BINARY PLY
        /*

        std::vector<float> data;
        const uint8_t* msg_data = msg_scan->data.data();
        
        for(size_t i=0; i< msg_scan->width * msg_scan->height; i++){
          float point = *((float*)msg_data);
          data.push_back(point);
          msg_data += sizeof(float);
          msg_data += sizeof(float);
          msg_data += sizeof(float);
          msg_data += sizeof(float);        
        }
        

        
        std::vector<float> x,y,z, intensity;
        int x_offset, y_offset, z_offset, intensity_offset;

        for(auto field : msg_scan->fields){
          
          if (field.name == "x"){
            x_offset = field.offset;
            x.resize(msg_scan->width * msg_scan->height);
          }

          else if (field.name == "y"){
            y_offset = field.offset;
            y.resize(msg_scan->width * msg_scan->height);
          }

          else if (field.name == "z"){
            z_offset = field.offset;
            z.resize(msg_scan->width * msg_scan->height);
          }

          else if (field.name == "intensity"){
            intensity_offset = field.offset;
            intensity.resize(msg_scan->width * msg_scan->height);
          }
        }


        std::memcpy(x.data(), &msg_scan->data[x_offset], x.size() * sizeof(float));
        std::memcpy(y.data(), &msg_scan->data[y_offset], y.size() * sizeof(float));
        std::memcpy(z.data(), &msg_scan->data[z_offset], z.size() * sizeof(float));
        std::memcpy(intensity.data(), &msg_scan->data[intensity_offset], intensity.size() * sizeof(float));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x= %f", x[0]);
        const uint8_t* data2 = msg_scan->data.data();
        float x_f = *((float*)(data2 + msg_scan->fields[0].offset));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "float x = %f", x_f);


        std::ofstream outFile(filePath, std::ios::out | std::ios::binary);

        outFile << "ply\n";
        outFile << "format binary_little_endian 1.0\n";
        //outFile << "format ascii 1.0\n";
        outFile << "element vertex " << msg_scan->width * msg_scan->height <<  "\n";
        outFile << "property float x\n";
        outFile << "property float y\n";
        outFile << "property float z\n";
        outFile << "property float intensity\n";
        outFile << "end_header\n";

        //outFile.write(reinterpret_cast<const char*>(&data[0]), data.size() * sizeof(float));



        for(uint32_t i=0; i < msg_scan->width * msg_scan->height; i++){
          outFile.write(reinterpret_cast<const char*>(&x[i]), sizeof(float));
          outFile.write(reinterpret_cast<const char*>(&y[i]), sizeof(float));
          outFile.write(reinterpret_cast<const char*>(&z[i]), sizeof(float));
          outFile.write(reinterpret_cast<const char*>(&intensity[i]), sizeof(float));
          //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "i %d", i);

        }

        outFile.close();

        */

  //////////////////
  ///ASCII PLY

        std::ofstream outFile(filePath, std::ios::out | std::ios::binary);

        outFile << "ply\n";
        //outFile << "format binary_little_endian 1.0\n";
        outFile << "format ascii 1.0\n";
        outFile << "comment pan angle " << pan_angle << "\n";
        outFile << "comment tilt angle " << tilt_angle << "\n";
        outFile << "comment lidar position x " << model_pose.position.x << "\n";
        outFile << "comment lidar position y " << model_pose.position.y << "\n";
        outFile << "comment lidar position z " << model_pose.position.z << "\n";
        outFile << "comment lidar orientation x " << model_pose.orientation.x << "\n";
        outFile << "comment lidar orientation y " << model_pose.orientation.y << "\n";
        outFile << "comment lidar orientation z " << model_pose.orientation.z << "\n";
        outFile << "comment lidar orientation w " << model_pose.orientation.w << "\n";
        outFile << "element vertex " << msg_scan->width * msg_scan->height <<  "\n";
        outFile << "property float x\n";
        outFile << "property float y\n";
        outFile << "property float z\n";
        outFile << "property uchar red\n";
        outFile << "property uchar green\n";
        outFile << "property uchar blue\n";
        //outFile << "property float intensity\n";
        if (param_normals_==1){
          outFile << "property float nx\n";
          outFile << "property float ny\n";
          outFile << "property float nz\n";
        }
        outFile << "end_header\n";




        const uint8_t* data = msg_scan->data.data();
        for (uint32_t i=0; i < msg_scan->width * msg_scan->height; i++){

          float x = *((float*)(data + msg_scan->fields[0].offset));

          if (x != x) x=0; 
          if (std::isinf(x)) x=0;
          //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "float x = %f", x);
        // float x_2 = *((char*)(data + msg_scan->fields[0].offset));
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "float x = %c", x_2);
          float y = *((float*)(data + msg_scan->fields[1].offset));
          if (y != y) y=0; 
          if (std::isinf(y)) y=0;
          float z = *((float*)(data + msg_scan->fields[2].offset));
          if (z != z) z=0;
          if (std::isinf(z)) z=0; 

          float RGB = *((float*)(data + msg_scan->fields[3].offset));

          uint32_t rgbValue = *reinterpret_cast<uint32_t*>(&RGB);

          uint8_t red = (rgbValue >> 16) & 0xFF;
          uint8_t green = (rgbValue >> 8) & 0xFF;
          uint8_t blue = rgbValue & 0xFF;

          // float red_v = *reinterpret_cast<float*>(&red);
          // float green_v= *reinterpret_cast<float*>(&green);
          // float blue_v= *reinterpret_cast<float*>(&blue);
          // float red_v = 255;
          // float green_v= 0;
          // float blue_v= 0;
          float red_v = int(red);
          float green_v= int(green);
          float blue_v= int(blue);


          

          if (param_normals_== 1){
            float nx = *((float*)(data + msg_scan->fields[4].offset));
            float ny = *((float*)(data + msg_scan->fields[5].offset));
            float nz = *((float*)(data + msg_scan->fields[6].offset));
            

            //outFile << x << " " << y << " " << z << " " << intensity << " " << nx << " " << ny << " " << nz << "\n";
            outFile << x << " " << y << " " << z << " " << red_v << " " << green_v << " " << blue_v << " " << nx << " " << ny << " " << nz << "\n";

            
          }
  
          else{
            //outFile << x << " " << y << " " << z << " " << intensity << "\n";
            outFile << x << " " << y << " " << z << " " << red_v << " " << green_v << " " << blue_v << "\n";

          }

          
          data += msg_scan->point_step;
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "point_step = %d", data);
        }

        outFile.close();
  //////////////////////////
      }

      else{



      }

      



    }

   
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_scan_;
    //rclcpp::Subscription<interfaces::msg::PosLidar>::SharedPtr subscription_pan_tilt_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_pan_angle_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_tilt_angle_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_model_pose_;
    
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PlyConverter>());



  rclcpp::shutdown();



  return 0;
}
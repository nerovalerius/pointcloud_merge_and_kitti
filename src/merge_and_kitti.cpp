/**
 * @file merge_and_kitti.cpp
 *
 * @brief Merges multiple pointcloud2 streams and stores the single images into pcd or binary files
 *
 * @author Armin Niedermueller
 * Contact: aniederm@mailbox.org
 *
 */

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>	
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/geometry_msgs/msg/twist.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/io/pcd_io.h>
#include <Eigen/Geometry>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <string>
#include <pcl/common/transforms.h>

using std::placeholders::_1;
using namespace std::chrono_literals;


class PointCloud2Subscriber : public rclcpp::Node
{
  public:

    PointCloud2Subscriber()
    : Node("pointcloud2_subscriber")
    {

        // create callback group for parallel execution of callbacks by ROS Executors
        pointcloud_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        options.callback_group = pointcloud_callback_group;

        timer_ = create_wall_timer(
        10ns, std::bind(&PointCloud2Subscriber::timer_callback, this));

        sub1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_1HDDGBM00101081",
                            10, std::bind(&PointCloud2Subscriber::cloud1_callback, this, _1));
        sub2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_1HDDH1200103541",
                            10, std::bind(&PointCloud2Subscriber::cloud2_callback, this, _1));
        sub3_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_3WEDH7600108991",
                            10, std::bind(&PointCloud2Subscriber::cloud3_callback, this, _1));
        sub4_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_3WEDJ5H00100311",
                            10, std::bind(&PointCloud2Subscriber::cloud4_callback, this, _1));
        sub5_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_3WEDJ5M00101211",
                            10, std::bind(&PointCloud2Subscriber::cloud5_callback, this, _1));

        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    }

  private:

    void cloud1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert from ROS Topic to point cloud object
        pcl::PointCloud<pcl::PointXYZI> temp_cloud_1, trans_temp_cloud_1;
        pcl::fromROSMsg(*msg, temp_cloud_1);
        transformStamped = tfBuffer->lookupTransform("livox_frame", "LivoxBroadcastCode1HDDGBM00101081", rclcpp::Time(0));

        // transform the point cloud into livox_frame
        //tf2::doTransform(temp_cloud_1, trans_temp_cloud_1, transformStamped);
        pcl::transformPointCloud(temp_cloud_1, trans_temp_cloud_1, tf2::transformToEigen(transformStamped).matrix(), true);
        
        // check
        if (this->clouds_received[0] == 0){
            this->full_cloud += trans_temp_cloud_1;
            this->clouds_received[0] = 1; 
        }
    }


    void cloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert from ROS Topic to point cloud object
        pcl::PointCloud<pcl::PointXYZI> temp_cloud_2, trans_temp_cloud_2;
        pcl::fromROSMsg(*msg, temp_cloud_2);

        transformStamped = tfBuffer->lookupTransform("livox_frame", "LivoxBroadcastCode1HDDH1200103541", rclcpp::Time(0));

        // transform the point cloud into livox_frame
        //tf2::doTransform(temp_cloud_2, trans_temp_cloud_2, transformStamped);
        pcl::transformPointCloud(temp_cloud_2, trans_temp_cloud_2, tf2::transformToEigen(transformStamped).matrix(), true);
        
        // check
        if (this->clouds_received[1] == 0){
            this->full_cloud += trans_temp_cloud_2;
            this->clouds_received[1] = 1; 
        }
    }


    void cloud3_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert from ROS Topic to point cloud object
        pcl::PointCloud<pcl::PointXYZI> temp_cloud_3, trans_temp_cloud_3;
        pcl::fromROSMsg(*msg, temp_cloud_3);

        transformStamped = tfBuffer->lookupTransform("livox_frame", "LivoxBroadcastCode3WEDJ5H00100311", rclcpp::Time(0));

        // transform the point cloud into livox_frame
        //tf2::doTransform(temp_cloud_3, trans_temp_cloud_3, transformStamped);
        pcl::transformPointCloud(temp_cloud_3, trans_temp_cloud_3, tf2::transformToEigen(transformStamped).matrix(), true);
        
        // check
        if (this->clouds_received[2] == 0){
            this->full_cloud += trans_temp_cloud_3;
            this->clouds_received[2] = 1; 
        }
    }


    void cloud4_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert from ROS Topic to point cloud object
        pcl::PointCloud<pcl::PointXYZI> temp_cloud_4, trans_temp_cloud_4;
        pcl::fromROSMsg(*msg, temp_cloud_4);

        transformStamped = tfBuffer->lookupTransform("livox_frame", "LivoxBroadcastCode3WEDH7600108991", rclcpp::Time(0));

        // transform the point cloud into livox_frame
        //tf2::doTransform(temp_cloud_4, trans_temp_cloud_4, transformStamped);
        pcl::transformPointCloud(temp_cloud_4, trans_temp_cloud_4, tf2::transformToEigen(transformStamped).matrix(), true);
        
        // check
        if (this->clouds_received[3] == 0){
            this->full_cloud += trans_temp_cloud_4;
            this->clouds_received[3] = 1; 
        }
    }


    void cloud5_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert from ROS Topic to point cloud object
        pcl::PointCloud<pcl::PointXYZI> temp_cloud_5, trans_temp_cloud_5;
        pcl::fromROSMsg(*msg, temp_cloud_5);

        transformStamped = tfBuffer->lookupTransform("livox_frame", "LivoxBroadcastCode3WEDJ5M00101211", rclcpp::Time(0));

        // transform the point cloud into livox_frame
        //tf2::doTransform(temp_cloud_5, trans_temp_cloud_5, transformStamped);
        pcl::transformPointCloud(temp_cloud_5, trans_temp_cloud_5, tf2::transformToEigen(transformStamped).matrix(), true);
        
        // check
        if (this->clouds_received[4] == 0){
            this->full_cloud += trans_temp_cloud_5;
            this->clouds_received[4] = 1; 
        }
    }


    void timer_callback()  {
        
        // check if 5 pointclouds are read in, thus, is the vector {1,1,1,1,1}?
        if (std::accumulate(this->clouds_received.begin(), this->clouds_received.end(), 0) == 5){
            // Save Pointcloud to file
            std::stringstream ss;
            ss << "./full_cloud_" << this->stored_full_clouds << ".pcd";
            std::string full_cloud_name = ss.str();           
            pcl::io::savePCDFileASCII(full_cloud_name, full_cloud);
            
            std::cout << full_cloud_name << " saved" << std::endl;
            this->clouds_timestamp = 0;

            // reset vector with point cloud status, thus {0,0,0,0,0}
            std::fill(this->clouds_received.begin(), this->clouds_received.end(), 0);
            this->stored_full_clouds++;

            // clear cloud
            this->full_cloud.clear();
        }

    }


    uint16_t clouds_timestamp = 0, stored_full_clouds = 0;
    std::vector<int> clouds_received{0,0,0,0,0};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    geometry_msgs::msg::TransformStamped transformStamped;
    rclcpp::CallbackGroup::SharedPtr pointcloud_callback_group;
    rclcpp::SubscriptionOptions options;
    bool full_cloud_completed;

    pcl::PointCloud<pcl::PointXYZI> full_cloud;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub1_, sub2_, sub3_, sub4_, sub5_;

};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    // Run Multithreaded for use in multihreaded callback group
    auto merge_node = std::make_shared<PointCloud2Subscriber>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(merge_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

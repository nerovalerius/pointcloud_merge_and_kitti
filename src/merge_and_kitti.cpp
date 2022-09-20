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
#include <queue>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>


#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
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
        pointcloud_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // alternative: threaded -> Reentrant
        options.callback_group = pointcloud_callback_group;

        timer_ = create_wall_timer(
        1ms, std::bind(&PointCloud2Subscriber::timer_callback, this));

        sub1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_1HDDGBM00101081",
                            10, std::bind(&PointCloud2Subscriber::cloud1_callback, this, _1), options);
        sub2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_1HDDH1200103541",
                            10, std::bind(&PointCloud2Subscriber::cloud2_callback, this, _1), options);
        sub3_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_3WEDH7600108991",
                            10, std::bind(&PointCloud2Subscriber::cloud3_callback, this, _1), options);
        sub4_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_3WEDJ5H00100311",
                            10, std::bind(&PointCloud2Subscriber::cloud4_callback, this, _1), options);
        sub5_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_3WEDJ5M00101211",
                            10, std::bind(&PointCloud2Subscriber::cloud5_callback, this, _1), options);

        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        // Store full point cloud id and separate timestamps of each pointcloud for deeper analysis
        cloud_csv.open ("full_clouds.csv");
        cloud_csv << "full cloud id; time cloud_1; time cloud_2; time cloud_3; time cloud_4; time cloud_5;\n";

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
        
        
        // Transformation Matrix for different steps of alignment
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        
        // Degrees
        int angle = -3;
        
        // Rotate  pointcloud by "angle" degrees in Z axis - its not perfectly aligned
        transform_1(0, 0) = cos(angle * M_PI / 180);
        transform_1(0, 1) = -sin(angle * M_PI / 180);
        transform_1(1, 0) = sin(angle * M_PI / 180);
        transform_1(1, 1) = cos(angle * M_PI / 180);
        transform_1(2, 2) = 1;
        transform_1(3, 3) = 1;
        
        // shift in axis
        transform_1(0, 3) = 0;      // x shift
        transform_1(1, 3) = 0.15;   // y shift
        transform_1(2, 3) = 0;      // z shift
        
        // Rotate fist pointcloud by "angle" degrees in Z axis
        pcl::transformPointCloud(trans_temp_cloud_1, trans_temp_cloud_1, transform_1, true);
        
        // add clouds in fifo queue
        double timestamp = (double)msg->header.stamp.sec + (double)(msg->header.stamp.nanosec / 1000000000.0f);
        clouds_1.push(std::make_tuple(trans_temp_cloud_1, timestamp));

    }


    void cloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert from ROS Topic to point cloud object
        pcl::PointCloud<pcl::PointXYZI> temp_cloud_2, trans_temp_cloud_2;
        pcl::fromROSMsg(*msg, temp_cloud_2);

        transformStamped = tfBuffer->lookupTransform("livox_frame", "LivoxBroadcastCode1HDDH1200103541", rclcpp::Time(0));

        // transform the point cloud into livox_frame
        //tf2::doTransform(temp_cloud_2, trans_temp_cloud_2, transformStamped);
        pcl::transformPointCloud(temp_cloud_2, trans_temp_cloud_2, tf2::transformToEigen(transformStamped).matrix(), true);
        
                // add clouds in fifo queue
        double timestamp = (double)msg->header.stamp.sec + (double)(msg->header.stamp.nanosec / 1000000000.0f);
        clouds_2.push(std::make_tuple(trans_temp_cloud_2, timestamp));
    }


    void cloud3_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert from ROS Topic to point cloud object
        pcl::PointCloud<pcl::PointXYZI> temp_cloud_3, trans_temp_cloud_3;
        pcl::fromROSMsg(*msg, temp_cloud_3);

        transformStamped = tfBuffer->lookupTransform("livox_frame", "LivoxBroadcastCode3WEDH7600108991", rclcpp::Time(0));

        // transform the point cloud into livox_frame
        //tf2::doTransform(temp_cloud_3, trans_temp_cloud_3, transformStamped);
        pcl::transformPointCloud(temp_cloud_3, trans_temp_cloud_3, tf2::transformToEigen(transformStamped).matrix(), true);
        
        // add clouds in fifo queue
        double timestamp = (double)msg->header.stamp.sec + (double)(msg->header.stamp.nanosec / 1000000000.0f);
        clouds_3.push(std::make_tuple(trans_temp_cloud_3, timestamp));
    }


    void cloud4_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert from ROS Topic to point cloud object
        pcl::PointCloud<pcl::PointXYZI> temp_cloud_4, trans_temp_cloud_4;
        pcl::fromROSMsg(*msg, temp_cloud_4);

        transformStamped = tfBuffer->lookupTransform("livox_frame", "LivoxBroadcastCode3WEDJ5H00100311", rclcpp::Time(0));

        // transform the point cloud into livox_frame
        //tf2::doTransform(temp_cloud_4, trans_temp_cloud_4, transformStamped);
        pcl::transformPointCloud(temp_cloud_4, trans_temp_cloud_4, tf2::transformToEigen(transformStamped).matrix(), true);
        
        // add clouds in fifo queue
        double timestamp = (double)msg->header.stamp.sec + (double)(msg->header.stamp.nanosec / 1000000000.0f);
        clouds_4.push(std::make_tuple(trans_temp_cloud_4, timestamp));
    }


    void cloud5_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert from ROS Topic to point cloud object
        pcl::PointCloud<pcl::PointXYZI> temp_cloud_5, trans_temp_cloud_5;
        pcl::fromROSMsg(*msg, temp_cloud_5);

        transformStamped = tfBuffer->lookupTransform("livox_frame", "LivoxBroadcastCode3WEDJ5M00101211", rclcpp::Time(0));

        // transform the point cloud into livox_frame
        //tf2::doTransform(temp_cloud_5, trans_temp_cloud_5, transformStamped);
        pcl::transformPointCloud(temp_cloud_5, trans_temp_cloud_5, tf2::transformToEigen(transformStamped).matrix(), true);
        
        // add clouds in fifo queue
        double timestamp = (double)msg->header.stamp.sec + (double)(msg->header.stamp.nanosec / 1000000000.0f);
        clouds_5.push(std::make_tuple(trans_temp_cloud_5, timestamp));
    }


    void timer_callback()  {

        // check if all the pointclouds have at least 1 cloud in their FIFO queue, thus we can merge all the newest clouds into one
        if (clouds_1.size() != 0 && clouds_2.size() != 0 && clouds_3.size() != 0 && clouds_4.size() != 0 && clouds_5.size() != 0 && allow_pointcloud_merge == true){

            // merge newest clouds from all streams
            full_cloud += std::get<0>(clouds_1.front());
            full_cloud += std::get<0>(clouds_2.front());
            full_cloud += std::get<0>(clouds_3.front());
            full_cloud += std::get<0>(clouds_4.front());
            full_cloud += std::get<0>(clouds_5.front());
            this->stored_full_clouds++;

            // store timestamps of clouds
            // CSV - full cloud id; time cloud_1; time cloud_2; time cloud_3; time cloud_4; time cloud_5;
            cloud_csv << this->stored_full_clouds << std::setprecision(9) << std::fixed \
                      << ";" << std::get<1>(clouds_1.front()) \
                      << ";" << std::get<1>(clouds_2.front()) \
                      << ";" << std::get<1>(clouds_3.front()) \
                      << ";" << std::get<1>(clouds_4.front()) \
                      << ";" << std::get<1>(clouds_5.front()) \
                      << ";\n";

            // check time difference of the point clouds
            double diff_12 = std::get<1>(clouds_1.front()) - std::get<1>(clouds_2.front());
            double diff_13 = std::get<1>(clouds_1.front()) - std::get<1>(clouds_3.front());
            double diff_14 = std::get<1>(clouds_1.front()) - std::get<1>(clouds_4.front());
            double diff_15 = std::get<1>(clouds_1.front()) - std::get<1>(clouds_5.front());
            double diff_23 = std::get<1>(clouds_2.front()) - std::get<1>(clouds_3.front());
            double diff_24 = std::get<1>(clouds_2.front()) - std::get<1>(clouds_4.front());
            double diff_25 = std::get<1>(clouds_2.front()) - std::get<1>(clouds_5.front());
            double diff_34 = std::get<1>(clouds_3.front()) - std::get<1>(clouds_4.front());
            double diff_35 = std::get<1>(clouds_3.front()) - std::get<1>(clouds_5.front());
            double diff_45 = std::get<1>(clouds_4.front()) - std::get<1>(clouds_5.front());
            double threshold = 0.2;

            if (diff_12 >= threshold || diff_13 >= threshold ||diff_14 >= threshold || diff_15 >= threshold\
               ||diff_23 >= threshold ||diff_24 >= threshold ||diff_25 >= threshold || diff_34 >= threshold\
               ||diff_35 >= threshold ||diff_45 >= threshold){
                
                std::cerr << "timestamp difference greater than 0.2 seconds occured!" << std::endl;

            }
    

            // remove clouds from FIFO Queue
            clouds_1.pop();
            clouds_2.pop();
            clouds_3.pop();
            clouds_4.pop();
            clouds_5.pop();

            // Save Pointcloud to file
            std::stringstream ss;
            ss << std::setw(6) << std::setfill('0') << this->stored_full_clouds << ".pcd";
            std::string full_cloud_name = ss.str();           
            pcl::io::savePCDFileASCII(full_cloud_name, full_cloud);
            
            std::cout << full_cloud_name << " saved" << std::endl;

            // clear cloud object
            this->full_cloud.clear();

            // close file if no pointclouds arrive anymore - threshold 10 seconds
            this->time_since_full_pointcloud = 0;

        } else {    // if no pointcloud arrives after a threshold of 5000 * 1 ms = 5 seconds, the file pointer is closed
            this->time_since_full_pointcloud++;
            if (this->time_since_full_pointcloud >= 5000){
                cloud_csv.close();
                std::cout << "no more pointclouds arrived since 5 seconds. Node is now shutdown" << std::endl;
                allow_pointcloud_merge = false;
                rclcpp::shutdown();
            }
        }

    }


    size_t stored_full_clouds = 0, time_since_full_pointcloud = 0;
    std::queue<std::tuple<pcl::PointCloud<pcl::PointXYZI>, double>> clouds_1, clouds_2, clouds_3, clouds_4, clouds_5;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    geometry_msgs::msg::TransformStamped transformStamped;
    rclcpp::CallbackGroup::SharedPtr pointcloud_callback_group;
    rclcpp::SubscriptionOptions options;
    bool allow_pointcloud_merge = true;
    std::ofstream cloud_csv;

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

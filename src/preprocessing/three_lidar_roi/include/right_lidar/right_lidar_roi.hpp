#ifndef __RIGHT_LIDAR_ROI_HPP__
#define __RIGHT_LIDAR_ROI_HPP__
#pragma once

// STD header
#include <string>
#include <stdlib.h>
#include <fstream>

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// Utility header
#include <util/ini_parser.hpp>

// Config header
#include <three_lidar_roi_config.hpp>

// Message header
#include <sensor_msgs/PointCloud2.h>

// carla
#include <carla_msgs/CarlaEgoVehicleStatus.h>

// PCL header
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class RightLidarRoi{
public:
    RightLidarRoi(ros::NodeHandle &nh_);
    ~RightLidarRoi();

    void Init();
    void ProcessINI();
    void Run();
    void Publish();
    void UpdateRviz();

    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr &in_lidar_msg);
    void VehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr &in_vehicle_status_msg);

    pcl::PCLPointCloud2 Ransac(const pcl::PCLPointCloud2 input_cloud);
    pcl::PCLPointCloud2 Voxelize(const pcl::PCLPointCloud2 input_cloud);
    pcl::PCLPointCloud2 AdaptiveROI(const pcl::PCLPointCloud2 input_cloud, double velocity);

    int GetRadius(double velocity);
    double GetHorizontalAngle(double x, double y);
    double GetVerticalAngle(double x, double y, double z);
    double GetVerticalROI(double velocity);

private:
    // Publisher
    ros::Publisher p_processed_lidar_pub;

    // Subscriber
    ros::Subscriber s_lidar_sub;
    ros::Subscriber s_vehicle_status_sub;

    // Environments
    IniParser v_ini_parser_;

    // Configuration parameters
    ThreeLidarRoiParameters three_lidar_roi_params_;   

    // Variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_raw_ptr;
    pcl::PCLPointCloud2 m_cloud_raw;
    sensor_msgs::PointCloud2 m_output;

    carla_msgs::CarlaEgoVehicleStatus m_vehicle_status;
    double m_velocity;

    int m_print_count = 0;

    // Vehicle Specifications
    float ego_vehicle_x_size = 5.2;  // length
    float ego_vehicle_y_size = 2.62; // width
    float ego_vehicle_z_size = 2.4;  // height

    float ego_vehicle_x = -2.7;                                  // in right lidar frame
    float ego_vehicle_y = 1.4;                                  // in right lidar frame


};

#endif // __RIGHT_LIDAR_ROI_HPP__
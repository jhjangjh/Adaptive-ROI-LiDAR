#ifndef __LEFT_LIDAR_ROI_HPP__
#define __LEFT_LIDAR_ROI_HPP__
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

class LeftLidarRoi{
public:
    LeftLidarRoi(ros::NodeHandle &nh_);
    ~LeftLidarRoi();

    void Init();
    void ProcessINI();
    void Run();
    void Publish();
    void UpdateRviz();

    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr &in_lidar_msg);

    pcl::PCLPointCloud2 Ransac(const pcl::PCLPointCloud2 cloud);

private:
    // Publisher
    ros::Publisher p_processed_lidar_pub;

    // Subscriber
    ros::Subscriber s_lidar_sub;

    // Environments
    IniParser v_ini_parser_;

    // Configuration parameters
    ThreeLidarRoiParameters three_lidar_roi_params_;   

    // Variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_raw_ptr;
    pcl::PCLPointCloud2 m_cloud_raw;
    sensor_msgs::PointCloud2 m_output;
};

#endif // __LEFT_LIDAR_ROI_HPP__
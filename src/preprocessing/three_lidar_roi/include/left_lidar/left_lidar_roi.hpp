#ifndef __LEFT_LIDAR_ROI_HPP__
#define __LEFT_LIDAR_ROI_HPP__
#pragma once

// STD header
#include <string>

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// Message header
#include <sensor_msgs/PointCloud2.h>

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

class LeftLidarRoi{
public:
    LeftLidarRoi(ros::NodeHandle &nh_);
    ~LeftLidarRoi();

    void Init();
    void Run();
    void Publish();
    void UpdateRviz();

    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr &in_lidar_msg);

private:
    // Publisher

    // Subscriber
    ros::Subscriber s_lidar_sub;
    
    // Variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_raw_ptr;

};

#endif // __LEFT_LIDAR_ROI_HPP__
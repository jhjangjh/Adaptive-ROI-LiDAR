#include <left_lidar/left_lidar_roi.hpp>

LeftLidarRoi::LeftLidarRoi(ros::NodeHandle &nh_){


    s_lidar_sub = nh_.subscribe("/carla/ego_vehicle/lidar_front_left", 10, &LeftLidarRoi::LidarCallback, this);

    Init();
}

LeftLidarRoi::~LeftLidarRoi(){}

void LeftLidarRoi::Init(){
    m_cloud_raw_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void LeftLidarRoi::Publish(){

}

void LeftLidarRoi::LidarCallback(const sensor_msgs::PointCloud2ConstPtr &in_lidar_msg){
    pcl::fromROSMsg(*in_lidar_msg,*m_cloud_raw_ptr);
}

void LeftLidarRoi::Run(){

}

void LeftLidarRoi::UpdateRviz(){

}

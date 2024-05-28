#include <left_lidar/left_lidar_roi.hpp>

LeftLidarRoi::LeftLidarRoi(ros::NodeHandle &nh_){

    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/lidar.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());

    s_lidar_sub = nh_.subscribe("/carla/ego_vehicle/lidar_front_left", 10, &LeftLidarRoi::LidarCallback, this);

    Init();
}

LeftLidarRoi::~LeftLidarRoi(){}

void LeftLidarRoi::Init(){
    ProcessINI();

    m_cloud_raw_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void LeftLidarRoi::ProcessINI(){
    if (v_ini_parser_.IsFileUpdated()){
        v_ini_parser_.ParseConfig("lidar", "voxel_size",
                                    three_lidar_roi_params_.voxel_size);
        ROS_WARN("[Lidar] Ini file is updated!\n");
    }
}

void LeftLidarRoi::Publish(){

}

void LeftLidarRoi::LidarCallback(const sensor_msgs::PointCloud2ConstPtr &in_lidar_msg){
    pcl::fromROSMsg(*in_lidar_msg,*m_cloud_raw_ptr);
}

void LeftLidarRoi::Run(){
    ProcessINI();
}

void LeftLidarRoi::UpdateRviz(){

}

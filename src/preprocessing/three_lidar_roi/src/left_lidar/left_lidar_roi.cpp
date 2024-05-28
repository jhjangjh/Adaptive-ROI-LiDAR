#include <left_lidar/left_lidar_roi.hpp>

LeftLidarRoi::LeftLidarRoi(ros::NodeHandle &nh_){

    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/lidar.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());

    p_processed_lidar_pub = nh_.advertise<pcl::PCLPointCloud2>("/processed_lidar",100);
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
        v_ini_parser_.ParseConfig("lidar", "ransac_threshold",
                                    three_lidar_roi_params_.ransac_threshold);                                    
        ROS_WARN("[Lidar] Ini file is updated!\n");
    }
}

void LeftLidarRoi::Publish(){
    p_processed_lidar_pub.publish(m_output);
}

void LeftLidarRoi::LidarCallback(const sensor_msgs::PointCloud2ConstPtr &in_lidar_msg){
    pcl::fromROSMsg(*in_lidar_msg,*m_cloud_raw_ptr);
    pcl::toPCLPointCloud2(*m_cloud_raw_ptr, m_cloud_raw);
}

void LeftLidarRoi::Run(){
    ProcessINI();
    pcl::PCLPointCloud2 ransac_cloud = Ransac(m_cloud_raw);
    pcl_conversions::fromPCL(ransac_cloud, m_output);
}

pcl::PCLPointCloud2 LeftLidarRoi::Ransac(const pcl::PCLPointCloud2 input_cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr before_processed_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground_removal (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(input_cloud, *before_processed_cloud_ptr);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointIndices::Ptr outliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true); // Optional

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(three_lidar_roi_params_.ransac_threshold);

    seg.setInputCloud(before_processed_cloud_ptr);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(before_processed_cloud_ptr);
    extract_indices.setIndices(inliers);
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud_ground_removal);

    pcl::PCLPointCloud2 output_cloud;
    pcl::toPCLPointCloud2(*cloud_ground_removal, output_cloud);

    return output_cloud;
}

void LeftLidarRoi::UpdateRviz(){

}

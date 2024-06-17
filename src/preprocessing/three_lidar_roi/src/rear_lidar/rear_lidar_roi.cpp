#include <rear_lidar/rear_lidar_roi.hpp>

RearLidarRoi::RearLidarRoi(ros::NodeHandle &nh_){

    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/preprocessing.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());

    p_processed_lidar_pub = nh_.advertise<pcl::PCLPointCloud2>("processed_lidar",100);

    s_lidar_sub = nh_.subscribe("/carla/ego_vehicle/lidar_rear", 10, &RearLidarRoi::LidarCallback, this);
    s_vehicle_status_sub = nh_.subscribe("/carla/ego_vehicle/vehicle_status", 10, &RearLidarRoi::VehicleStatusCallback, this);

    Init();
}

RearLidarRoi::~RearLidarRoi(){}

void RearLidarRoi::Init(){
    ProcessINI();

    m_cloud_raw_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void RearLidarRoi::ProcessINI(){
    if (v_ini_parser_.IsFileUpdated()){
        v_ini_parser_.ParseConfig("rear_lidar", "ransac_threshold",
                                    three_lidar_roi_params_.ransac_threshold);                                    
        v_ini_parser_.ParseConfig("rear_lidar", "voxel_size",
                                    three_lidar_roi_params_.voxel_size);
        v_ini_parser_.ParseConfig("rear_lidar", "number_of_lanes",
                                    three_lidar_roi_params_.number_of_lanes);
        v_ini_parser_.ParseConfig("rear_lidar", "radius",
                                    three_lidar_roi_params_.radius);
        ROS_WARN("[Preprocessing] Ini file is updated!\n");
    }
}

void RearLidarRoi::Publish(){
    m_output.header.frame_id = "ego_vehicle/lidar_rear";
    p_processed_lidar_pub.publish(m_output);
}

void RearLidarRoi::LidarCallback(const sensor_msgs::PointCloud2ConstPtr &in_lidar_msg){
    pcl::fromROSMsg(*in_lidar_msg,*m_cloud_raw_ptr);
    pcl::toPCLPointCloud2(*m_cloud_raw_ptr, m_cloud_raw);
}

void RearLidarRoi::VehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr &in_vehicle_status_msg){
    m_vehicle_status = *in_vehicle_status_msg;
    m_velocity = m_vehicle_status.velocity;
}

void RearLidarRoi::Run(){
    ProcessINI();

    if(m_cloud_raw_ptr->size() != 0)
    {
        // RANSAC
        pcl::PCLPointCloud2 ransac_cloud = Ransac(m_cloud_raw);
        // pcl_conversions::fromPCL(ransac_cloud, m_output);
        
        // Voxelize
        pcl::PCLPointCloud2 voxel_cloud = Voxelize(ransac_cloud);
        // pcl_conversions::fromPCL(voxel_cloud, m_output);


        // Adaptive ROI processing
        // pcl::PCLPointCloud2 adapive_roi_cloud = AdaptiveROI(m_cloud_raw,m_velocity);
        pcl::PCLPointCloud2 adapive_roi_cloud = AdaptiveROI(voxel_cloud,m_velocity);
        
        pcl_conversions::fromPCL(adapive_roi_cloud, m_output);


        if(m_print_count++ % 20 == 0)
        {
            ROS_INFO_STREAM("Rear LiDAR Processing...");
        }
    }
    else
    {
        ROS_WARN_STREAM("No Rear LiDAR Point!!");
    }

    UpdateRviz();
}

pcl::PCLPointCloud2 RearLidarRoi::Ransac(const pcl::PCLPointCloud2 input_cloud){
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
    extract_indices.setNegative(true); // true : erase inlier , false : erase outlier
    extract_indices.filter(*cloud_ground_removal);

    pcl::PCLPointCloud2 output_cloud;
    pcl::toPCLPointCloud2(*cloud_ground_removal, output_cloud);

    return output_cloud;
}

pcl::PCLPointCloud2 RearLidarRoi::Voxelize(const pcl::PCLPointCloud2 input_cloud){
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(input_cloud, *input_cloud_ptr);

    double voxelsize = three_lidar_roi_params_.voxel_size;

    voxel_filter.setInputCloud(input_cloud_ptr);
    voxel_filter.setLeafSize(voxelsize,voxelsize,voxelsize);
    voxel_filter.filter(*voxelized_ptr);

    pcl::PCLPointCloud2 output;
    pcl::toPCLPointCloud2(*voxelized_ptr,output);

    return output;
}

pcl::PCLPointCloud2 RearLidarRoi::AdaptiveROI(const pcl::PCLPointCloud2 input_cloud, double velocity){
    // Move lidar center point (by KDTree Radius Search)
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(input_cloud, *input_cloud_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    std::vector<int> idxes;
    std::vector<float> sqr_dists;

    pcl::PointXYZ query_point(ego_vehicle_x, ego_vehicle_y, ego_vehicle_z_size / 2);

    int radius = GetRadius(velocity);

    kdtree.setInputCloud(input_cloud_ptr);
    kdtree.radiusSearch(query_point, radius, idxes, sqr_dists);
    for (const auto& idx: idxes){
        boundary->points.push_back(input_cloud_ptr->points[idx]);
    }

    // Filter points by road width
    float max_road_width = 3.25 * three_lidar_roi_params_.number_of_lanes/2;  // one lane width = 3.25 [m]

    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud(boundary);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-max_road_width / 2. , max_road_width / 2.);
    pass.filter(*boundary);

    // ROI Angle Setting
    for(unsigned int j=0; j<boundary->points.size(); j++)
    {
        double angle_h = GetHorizontalAngle((boundary->points[j].x - ego_vehicle_x) , (boundary->points[j].y - ego_vehicle_y));
        double angle_v = GetVerticalAngle(boundary->points[j].x , boundary->points[j].y , boundary->points[j].z);

        if(angle_h < 153.3) // 153.3 = 180 - degree(arctan(1.31/2.6)) 
        {
            boundary->points[j].x = ego_vehicle_x;
            boundary->points[j].y = ego_vehicle_y;
            boundary->points[j].z = 0;
        }

        if(angle_v > GetVerticalROI(velocity)) // 153.3 = 180 - degree(arctan(1.31/2.6)) 
        {
            boundary->points[j].x = ego_vehicle_x;
            boundary->points[j].y = ego_vehicle_y;
            boundary->points[j].z = 0;
        }

    }    

    // Filter points reflected by ego_vehicle points
    pcl::PassThrough<pcl::PointXYZ> pass_vehicle_point;

    pcl::PointCloud<pcl::PointXYZ>::Ptr outskirt(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pass_vehicle_point.setInputCloud(boundary);
    pass_vehicle_point.setFilterFieldName("x");
    pass_vehicle_point.setFilterLimits(0.0,5.4);
    pass_vehicle_point.setFilterLimitsNegative(true);
    pass_vehicle_point.filter(*outskirt);

    pass_vehicle_point.setInputCloud(boundary);
    pass_vehicle_point.setFilterFieldName("y");
    pass_vehicle_point.setFilterLimits(-1.4,1.4);
    pass_vehicle_point.setFilterLimitsNegative(true);
    pass_vehicle_point.filter(*output_ptr);

    *output_ptr += *outskirt;

    pcl::PCLPointCloud2 output;
    pcl::toPCLPointCloud2(*output_ptr,output);

    return output;

}

void RearLidarRoi::UpdateRviz(){

}

int RearLidarRoi::GetRadius(double velocity){  // TBD
    float time_delay = 0.5;
    float time_detect = 0.3;
    float time_safe = 1.;
    float times = time_delay + time_detect + time_safe;

    // int radius = round(velocity * times);
    int radius = three_lidar_roi_params_.radius;
    return radius;
}

double RearLidarRoi::GetHorizontalAngle(double x, double y)
{
    double r;
    double theta;

    r = sqrt((x*x)+(y*y));
    theta = acos(x/r)*180/M_PI;
    return theta;
}

double RearLidarRoi::GetVerticalAngle(double x, double y, double z)
{
    double r;
    double theta;

    r = sqrt((x*x)+(y*y));
    theta = atan(z/r)*180/M_PI;
    return theta;
}

double RearLidarRoi::GetVerticalROI(double velocity){  // TBD
    double angle_limit;

    if (velocity < 60){
        angle_limit = 22.5;
    }
    else{
        angle_limit = 5.7 - (0.2 * (velocity - 60));
    }
    return angle_limit;
}
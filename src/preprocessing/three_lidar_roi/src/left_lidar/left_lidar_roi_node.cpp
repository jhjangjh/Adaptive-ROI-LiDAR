#include <left_lidar/left_lidar_roi.hpp>

int main(int argc, char** argv)
{
    std::string node_name = "left_lidar_roi_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~");

    LeftLidarRoi left_lidar_roi(nh);
    ros::Rate loop_rate(30);

    while(ros::ok())
    {   
        left_lidar_roi.Run();
        left_lidar_roi.Publish();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
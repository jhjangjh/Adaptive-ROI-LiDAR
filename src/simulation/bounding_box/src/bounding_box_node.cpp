#include <bounding_box.hpp>

int main(int argc, char** argv)
{
    std::string node_name = "bounding_box_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~");

    Bounding bounding_box(nh);
    ros::Rate loop_rate(30);

    while(ros::ok())
    {   
        bounding_box.Publish();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
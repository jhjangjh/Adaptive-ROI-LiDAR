// ROS header
#include <ros/ros.h>
#include <tf/tf.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

class Bounding{
public:
    Bounding(ros::NodeHandle &nh_);
    ~Bounding();

    void Publish();

    void MarkerarrayCallback(const visualization_msgs::MarkerArrayConstPtr &markerarray_msg);

    jsk_recognition_msgs::BoundingBox markerToBoundingBox(const visualization_msgs::Marker& marker);

private:
    // Publisher
    ros::Publisher bounding_box_pub;

    // Subscriber
    ros::Subscriber markerarray_sub;

    visualization_msgs::MarkerArray temp;

};

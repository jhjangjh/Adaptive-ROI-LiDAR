#include <bounding_box.hpp>

Bounding::Bounding(ros::NodeHandle &nh_){

    bounding_box_pub = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("bounding_box",100);

    markerarray_sub = nh_.subscribe("/carla/markers", 10, &Bounding::MarkerarrayCallback, this);

}

Bounding::~Bounding(){}


void Bounding::Publish(){
    jsk_recognition_msgs::BoundingBoxArray bound_box_arr;
    bound_box_arr.header.frame_id = "map";
    // temp.markers.erase(temp.markers.begin());

    bool first = true;

    for (std::vector<visualization_msgs::Marker>::iterator itr = temp.markers.begin(); itr != temp.markers.end(); ++itr)
    {
        if(first)
        {
            first = false;
            continue;
        }
        bound_box_arr.boxes.push_back(markerToBoundingBox(*itr));
    }

    bounding_box_pub.publish(bound_box_arr);
}

void Bounding::MarkerarrayCallback(const visualization_msgs::MarkerArray::ConstPtr &markerarray_msg){
    temp = *markerarray_msg;
}

jsk_recognition_msgs::BoundingBox Bounding::markerToBoundingBox(const visualization_msgs::Marker& marker) {
    jsk_recognition_msgs::BoundingBox bbox;
    bbox.header = marker.header;
    bbox.pose = marker.pose;
    bbox.dimensions.x = marker.scale.x;
    bbox.dimensions.y = marker.scale.y;
    bbox.dimensions.z = marker.scale.z;
    return bbox;
}
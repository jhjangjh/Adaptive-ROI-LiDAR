/*
 *  Copyright (c) 2020, ADASONE
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MULTI_LIDAR_COMBINER_H
#define MULTI_LIDAR_COMBINER_H

#define PCL_NO_PRECOMPILE

#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint16_t, ring, ring)
)

namespace multi_lidar_combiner {

typedef pcl::PointCloud<Point> PointCloudT;
typedef sensor_msgs::PointCloud2 PointCloudMsgT;
typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                        PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                        PointCloudMsgT, PointCloudMsgT> SyncPolicyT;

class MultiLidarCombiner {
public:
  MultiLidarCombiner();
  ~MultiLidarCombiner();

  void Run();

private:
  void GetParam();
  void CreateIo();
  void GetTfData();
  void LidarCallback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                     const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                     const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                     const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8);

  void RemoveVehiclePoints(pcl::PointCloud<Point>::Ptr cloud);
  void DownsampleCloud(pcl::PointCloud<Point>::ConstPtr in_cloud_ptr,
                        pcl::PointCloud<Point>::Ptr out_cloud_ptr, double in_leaf_size);

  ros::NodeHandle nh_{""};
  ros::NodeHandle pnh_{"~"};

  ros::Publisher pub_cloud_;  // preprocess cloud

  message_filters::Subscriber<PointCloudMsgT> *cloud_subscribers_[8];
  message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;

  tf::TransformListener listener_;

  tf::StampedTransform base2lslidar1_transform_;
  tf::StampedTransform base2lslidar2_transform_;

  std::string frame_id_;
  size_t input_topics_size_;
  std::string input_topics_;
  std::string topic_name_;
  YAML::Node topics_;
  bool debug_;

};

}  // namespace multi_lidar_combiner
#endif  // end MULTI_LIDAR_COMBINER_H

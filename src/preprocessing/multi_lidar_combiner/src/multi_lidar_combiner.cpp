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

#include "multi_lidar_combiner.h"

namespace multi_lidar_combiner {

MultiLidarCombiner::MultiLidarCombiner() {
  // Geting parameter
  GetParam();
  // Create ros Io
  CreateIo();
  // GetTfData(); 
}
MultiLidarCombiner::~MultiLidarCombiner() {}

void MultiLidarCombiner::GetParam() {
  // frame param
  pnh_.param("input_topics", input_topics_, std::string("[/points_alpha, /points_beta]"));
  pnh_.param("frame_id", frame_id_, std::string("lidar_link"));
  pnh_.param("topic_name", topic_name_, std::string("combin_cloud"));
  pnh_.param("debug", debug_, bool(false));

  topics_ = YAML::Load(input_topics_);
  input_topics_size_ = topics_.size();
  if (input_topics_size_ < 1 || 8 < input_topics_size_)
  {
    ROS_ERROR("The size of input_topics must be between 2 and 8");
    ros::shutdown();
  }
}

void MultiLidarCombiner::CreateIo() {
  pub_cloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>(topic_name_.c_str(), 1);

  for (size_t i = 0; i < 8; ++i) {
    if (i < input_topics_size_) {
      cloud_subscribers_[i] =
          new message_filters::Subscriber<PointCloudMsgT>(nh_, topics_[i].as<std::string>(), 1, ros::TransportHints().tcpNoDelay());
    } else {
      cloud_subscribers_[i] =
          new message_filters::Subscriber<PointCloudMsgT>(nh_, topics_[0].as<std::string>(), 1, ros::TransportHints().tcpNoDelay());
    }
  }

  cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
      SyncPolicyT(20), *cloud_subscribers_[0], *cloud_subscribers_[1], *cloud_subscribers_[2], *cloud_subscribers_[3],
      *cloud_subscribers_[4], *cloud_subscribers_[5], *cloud_subscribers_[6], *cloud_subscribers_[7]);
  cloud_synchronizer_->registerCallback(
      boost::bind(&MultiLidarCombiner::LidarCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));
}

void MultiLidarCombiner::GetTfData() {
  bool flag = true;

  // 이 딜레이는 아래 lookupTransform오류를 방지하기 위해서 이다. 
  ros::Duration(1.0).sleep();
  
  while (flag) {
    try{
      listener_.lookupTransform(frame_id_.c_str(), "laser_link_left",  
                              ros::Time(0), base2lslidar1_transform_);
      listener_.lookupTransform(frame_id_.c_str(), "laser_link_right",  
                              ros::Time(0), base2lslidar2_transform_);
      flag = false;
    } catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ROS_ERROR("not found lidar tf");
      ros::Duration(1.0).sleep();
    }
  }
}

void MultiLidarCombiner::LidarCallback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                       const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                       const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                       const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8) {
  ros::Time start_time = ros::Time::now();

  assert(2 <= input_topics_size_ && input_topics_size_ <= 8);

  PointCloudMsgT::ConstPtr msgs[8] = { msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8 };
  PointCloudT::Ptr cloud_sources[8];
  PointCloudT::Ptr cloud_concatenated(new PointCloudT);

  // transform points
  try {
    for (size_t i = 0; i < input_topics_size_; ++i) {
      // Note: If you use kinetic, you can directly receive messages as
      // PointCloutT.
      tf::StampedTransform base2lidar_transform;
      Eigen::Matrix4f eigen_base2lidar_transform;

      cloud_sources[i] = PointCloudT().makeShared();
      
      pcl::fromROSMsg(*msgs[i], *cloud_sources[i]);
      
      //TODO::  36번 전용 rear lidar 64ch > 16ch  
      if (i == 0) {
        PointCloudT::Ptr temp_cloud = PointCloudT().makeShared();
        
        for (auto point : cloud_sources[i]->points) {
          if (point.ring % 4 == 0) {
            temp_cloud->points.push_back(point);
          }
        }

        *cloud_sources[i] = *temp_cloud; 
      }


      listener_.lookupTransform(frame_id_, msgs[i]->header.frame_id,  
                              ros::Time(0), base2lidar_transform);
      pcl_ros::transformAsMatrix(base2lidar_transform, eigen_base2lidar_transform);
      pcl::transformPointCloud(*cloud_sources[i], *cloud_sources[i], eigen_base2lidar_transform);
    }
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // merge points
  for (size_t i = 0; i < input_topics_size_; ++i) {
    *cloud_concatenated += *cloud_sources[i];
  }

  // DownsampleCloud(cloud_concatenated, cloud_concatenated, 0.1);
  // RemoveVehiclePoints(cloud_concatenated);
  
  // pcl::PassThrough<Point> pass;
  // pass.setInputCloud(cloud_concatenated);                //입력 
  // pass.setFilterFieldName("z");             //적용할 좌표 축 (eg. Z축)
  // pass.setFilterLimits (-0.6, 2);          //적용할 값 (최소, 최대 값)
  // // pass.setFilterLimitsNegative (true);     //적용할 값 외 
  // pass.filter (*cloud_concatenated);             //필터 적용 


  ros::Time end_time = ros::Time::now();
  if (debug_) {
    ROS_INFO("full time : %f ms", (end_time - start_time).toSec() * 1000.0);
  }
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_concatenated, cloud_msg);
  cloud_msg.header.frame_id = frame_id_;
  cloud_msg.is_dense = true;
  //  cloud_msg.header.stamp = msgs[0]->header.stamp;
  // cloud_msg.header.stamp = msgs[0]->header.stamp;
  cloud_msg.header.stamp = ros::Time::now();
  pub_cloud_.publish(cloud_msg);
}

void MultiLidarCombiner::RemoveVehiclePoints(pcl::PointCloud<Point>::Ptr cloud) {
  pcl::PointCloud<Point>::Ptr filtered_cloud(new pcl::PointCloud<Point>);

  pcl::ConditionOr<Point>::Ptr range_cond(new pcl::ConditionOr<Point> ());
  // if point.x > -8.6 && point.x < 0.1 && point.y > -1.2 && point.y < 1.2
  range_cond->addComparison(pcl::FieldComparison<Point>::ConstPtr(
      new pcl::FieldComparison<Point> ("x", pcl::ComparisonOps::LT, -8)));
  range_cond->addComparison(pcl::FieldComparison<Point>::ConstPtr(
      new pcl::FieldComparison<Point> ("x", pcl::ComparisonOps::GT, 0.3)));
  range_cond->addComparison(pcl::FieldComparison<Point>::ConstPtr(
      new pcl::FieldComparison<Point> ("y", pcl::ComparisonOps::LT, -1.3)));
  range_cond->addComparison(pcl::FieldComparison<Point>::ConstPtr(
      new pcl::FieldComparison<Point> ("y", pcl::ComparisonOps::GT, 1.3)));
  
  // build the filter
  pcl::ConditionalRemoval<Point> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(cloud);
  condrem.setKeepOrganized(true);
  // apply filter
  condrem.filter(*cloud);
}

void MultiLidarCombiner::DownsampleCloud(
    pcl::PointCloud<Point>::ConstPtr in_cloud_ptr,
    pcl::PointCloud<Point>::Ptr out_cloud_ptr, double in_leaf_size) {
  pcl::VoxelGrid<Point> voxelized;
  voxelized.setInputCloud(in_cloud_ptr);
  voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size,
                        (float)in_leaf_size);
  voxelized.filter(*out_cloud_ptr);
}

void MultiLidarCombiner::Run() {
  ros::spin();
}

} // namespace multi_lidar_combiner

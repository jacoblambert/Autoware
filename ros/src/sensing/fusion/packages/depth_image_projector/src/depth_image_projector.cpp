/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
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
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************
 *  v1.0: Jacob Lambert (jacob.lambert@tier4.jp)
 *
 * depth_image_projector.cpp
 *
 *  Created on: Jan 25th, 2018
 */

#include "depth_image_projector/depth_image_projector.h"

pcl::PointXYZ
RosDepthImageProjectorApp::TransformPoint(const pcl::PointXYZ &in_point, const geometry_msgs::TransformStamped &in_transform)
{
  geometry_msgs::Point point, point_transformed;

  point.x = in_point.x;
  point.y = in_point.y;
  point.z = in_point.z;

  tf2::doTransform(point, point_transformed, in_transform);

  return pcl::PointXYZ(point_transformed.x, point_transformed.y, point_transformed.z);
}

void RosDepthImageProjectorApp::DepthImageCallback(const autoware_msgs::DepthImage::ConstPtr &in_depth_msg)
{
  if (!camera_lidar_tf_ok_)
  {
    camera_lidar_tf_ = FindTransform(cloud_frame_id_, in_depth_msg->header.frame_id);
  }
  if (!camera_info_ok_ || !camera_lidar_tf_ok_)
  {
    ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
    return;
  }
  min_depth_ = in_depth_msg->min_depth;
  max_depth_ = in_depth_msg->max_depth;
  cv::Mat depth_image;
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_depth_msg->depth_image); //
  depth_image = cv_image->image;
  image_size_ = depth_image.size();

  cv::Mat normalized_depth_image(image_size_.height, image_size_.width, CV_32FC1);

  cv::normalize(depth_image, normalized_depth_image, max_depth_, min_depth_, cv::NORM_MINMAX, CV_32FC1);

  pcl::PointCloud<pcl::PointXYZ> depth_cloud;

  for (size_t i = 0; i < image_size_.width; i++) {
    for (size_t j = 0; j < image_size_.height; j++) {
//      std::cout << "Normalize (" << i << ", " << j << "), " << image_size_.height << ", " << image_size_.width << ")\n";
      float z = normalized_depth_image.at<float>(j,i);
      if (z > min_depth_) {
//        std::cout << "Get Pt\n";
        pcl::PointXYZ cam_point, cloud_point;
        cam_point.x = (i - cx_) * z / fx_;
        cam_point.y = (j - cy_) * z / fy_;
        cam_point.z = z;
//        std::cout << "TF\n";
        cloud_point = TransformPoint(cam_point, camera_lidar_tf_);
//        std::cout << "Push-backF\n";
        depth_cloud.push_back(cloud_point);
      }
    }
  }
  sensor_msgs::PointCloud2 out_cloud_msg;
  pcl::toROSMsg(depth_cloud, out_cloud_msg);
  out_cloud_msg.header = in_depth_msg->header;
  out_cloud_msg.header.frame_id = cloud_frame_id_.c_str();
  cloud_publisher_.publish(out_cloud_msg);

}

void RosDepthImageProjectorApp::IntrinsicsCallback(const sensor_msgs::CameraInfo &in_message)
{
  image_size_.height = in_message.height;
  image_size_.width = in_message.width;

  camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
    }
  }

  distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
  for (int col = 0; col < 5; col++)
  {
    distortion_coefficients_.at<double>(col) = in_message.D[col];
  }

  fx_ = static_cast<float>(in_message.P[0]);
  fy_ = static_cast<float>(in_message.P[5]);
  cx_ = static_cast<float>(in_message.P[2]);
  cy_ = static_cast<float>(in_message.P[6]);

  intrinsics_subscriber_.shutdown();
  camera_info_ok_ = true;
  //image_frame_id_ = in_message.header.frame_id;
  ROS_INFO("[%s] CameraIntrinsics obtained.", __APP_NAME__);
}

geometry_msgs::TransformStamped
RosDepthImageProjectorApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
  geometry_msgs::TransformStamped transform;

  camera_lidar_tf_ok_ = false;
  try
  {
    transform = tf_buffer_.lookupTransform(in_target_frame, in_source_frame, ros::Time(0));
    camera_lidar_tf_ok_ = true;
    ROS_INFO("[%s] Camera-Lidar TF obtained", __APP_NAME__);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
  }

  return transform;
}

void RosDepthImageProjectorApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
  //get params
  std::string points_src, depth_image_src, camera_info_src;
  std::string cloud_topic_str = "/depth_cloud";
  std::string name_space_str = ros::this_node::getNamespace();

  ROS_INFO("[%s] This node requires: Registered TF(Lidar-Camera), CameraInfo, Image, and PointCloud.", __APP_NAME__);
  in_private_handle.param<std::string>("depth_image_src", depth_image_src, "/depth_image");
  ROS_INFO("[%s] depth_image_src: %s", __APP_NAME__, depth_image_src.c_str());

  in_private_handle.param<std::string>("camera_info_src", camera_info_src, "/camera_info");
  ROS_INFO("[%s] camera_info_src: %s", __APP_NAME__, camera_info_src.c_str());

  if (name_space_str != "/")
  {
    if (name_space_str.substr(0, 2) == "//")
    {
      name_space_str.erase(name_space_str.begin());
    }
    depth_image_src = name_space_str + depth_image_src;
    camera_info_src = name_space_str + camera_info_src;
  }

  //generate subscribers and synchronizers
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, camera_info_src.c_str());
  intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src,
                                                       1,
                                                       &RosDepthImageProjectorApp::IntrinsicsCallback, this);

  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, depth_image_src.c_str());
  depth_image_subscriber_ = in_private_handle.subscribe(depth_image_src,
                                                       1,
                                                       &RosDepthImageProjectorApp::DepthImageCallback, this);


  cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(cloud_topic_str, 1);
  ROS_INFO("[%s] Publishing projected pointcloud in %s", __APP_NAME__, cloud_topic_str.c_str());


}

void RosDepthImageProjectorApp::Run()
{
  ros::NodeHandle private_node_handle("~");

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener transform_listener(tf_buffer);

  InitializeRosIo(private_node_handle);

  ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

  ros::spin();

  ROS_INFO("[%s] END", __APP_NAME__);
}

RosDepthImageProjectorApp::RosDepthImageProjectorApp() : transform_listener_(tf_buffer_)
{
  camera_lidar_tf_ok_ = false;
  camera_info_ok_ = false;
  processing_ = false;
  cloud_frame_id_ = "velodyne"; // TODO: set this in launch file?
}
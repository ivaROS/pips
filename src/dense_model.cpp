/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Suat Gedikli */

#include "hallucinated_robot_model.h"
#include <mesh_filter/depth_model.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_datatypes.h>

#include <memory>




DenseModel::DenseModel(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    nh_(nh), 
    pnh_(pnh)
{
  tfBuffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, nh_);
  depth_model_ = std::make_shared<depth_projection::DepthModel>(tfBuffer_);
  depth_model_->init("robot_description");
}



bool DenseModel::isReady()
{
    
  if(!cam_model_)
  {
    ROS_WARN("Cannot render depth image until camera info is received!");
    return false;
  }
  
  if(!depth_model_->isReady())
  {
    return false;
  }
  
  return true;
}


bool DenseModel::testCollisionImpl(const geometry_msgs::Pose pose)
{
  cv::Mat model_depth = generateHallucinatedRobot(pose);
  
  if(cv::countNonZero(model_depth < cv_image_ref_->image) > 0)
  {
    return true;
  }
  return false;
}

cv::Mat DenseModel::generateHallucinatedRobotImpl(const geometry_msgs::Pose pose)
{
  if(isReady())
  {
    const geometry_msgs::Pose::ConstPtr pose_ptr(new geometry_msgs::Pose(pose));  // Note: if depth_model_ took in normal pointers to pose, etc, I could just pass in the address rather than creating a new object. Is there any reason not to do that?
    const sensor_msgs::Image::ConstPtr img_msg = depth_model_->generateDepthModel(pose_ptr, cv_image_ref_, cam_model_->cameraInfo());
    
    if(img_msg)
    {
      cv_bridge::CvImage::ConstPtr img_cv = cv_bridge::toCvShare(img_msg);
      return img_cv->image;
    }
  }
    
  return cv::Mat();
}






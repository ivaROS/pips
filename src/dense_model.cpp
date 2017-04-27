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

#include "opencv2/highgui/highgui.hpp"


DenseModel::DenseModel(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    HallucinatedRobotModelImpl(),
    nh_(nh), 
    pnh_(pnh)
{
  tfBuffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, nh_);
  depth_model_ = std::make_shared<depth_projection::DepthModel>(tfBuffer_);
  depth_model_->init("robot_description");//, "camera_depth_optical_frame", "base_footprint");
  
  model_depth_transport_ = std::make_shared<image_transport::ImageTransport>(nh_);
  
  pub_model_depth_image_ = model_depth_transport_->advertiseCamera("model_depth", 1);
}

DenseModel::~DenseModel() {}

bool DenseModel::isReady()
{
    
  if(!cam_model_)
  {
    ROS_WARN_NAMED(name_, "Cannot render depth image until camera info is received!");
    return false;
  }
  
  if(!depth_model_->isReady())
  {
    return false;
  }
  
  return true;
}


bool DenseModel::isLessThan(const cv::Mat& depth_im, const cv::Mat& generated_im)
{
/*
    double min_depth;
    cv::minMaxLoc(image, &min_depth, NULL, NULL, NULL);

    return min_depth < depth;
    */
    
    //Built in approach:
/*
    cv::Mat collisions = (roi <= co_depth*scale_);
    int num_collisions = cv::countNonZero(collisions);
    bool collided = (num_collisions>0);
  */
    
    int nRows = depth_im.rows;
    int nCols = depth_im.cols;

    bool retVal = false;
    
    //Could use templates to remove the duplication of this code
    if(depth_im.depth() == CV_32FC1)
    {
      int i,j;
      const float* p_d;
      const float* p_g;
      for( i = 0; i < nRows; ++i)
      {
          p_d = depth_im.ptr<float>(i);
          p_g = generated_im.ptr<float>(i);
          for ( j = 0; j < nCols; ++j)
          {
              if(p_d[j] < p_g[j])
              {
                retVal = true;
                std::cout << "depth["<< j << "," << i << "] = " << p_d[j] << " < " << p_g[j] << "\n";
              }
                
          }
      }
    }
    else if (depth_im.depth() == CV_16UC1)
    {
      int i,j;
      const unsigned short int* p_d;
      const unsigned short int* p_g;
      for( i = 0; i < nRows; ++i)
      {
          p_d = depth_im.ptr<unsigned short int>(i);
          p_g = generated_im.ptr<unsigned short int>(i);
          for ( j = 0; j < nCols; ++j)
          {
              if(p_d[j] < p_g[j])
              {
                return true;
              }
                
          }
      }
    }
    
    return retVal;
}

bool DenseModel::testCollisionImpl(const geometry_msgs::Pose pose)
{
  cv::Mat model_depth = generateHallucinatedRobotImpl(pose);
  
  cv::Mat diff = model_depth > cv_image_ref_->image;
  
  cv::Mat nans = cv::Mat(model_depth != model_depth);
  int num_nans = cv::countNonZero(nans);
  
  if(num_nans > 0)
    std::cout << "# nans in model image: " << num_nans << "\n";
    
  //nans = cv::Mat(cv_image_ref_->image != cv_image_ref_->image);
  
  //num_nans = cv::countNonZero(nans);
  
  //if(num_nans > 0)
  //  std::cout << "# nans in depth image: " << num_nans << "\n";
    
  bool collision = false;
  
  bool cvRes = (cv::countNonZero(diff) > 0);
  bool myRes = isLessThan(cv_image_ref_->image, model_depth);
  
  if(cvRes || myRes)
  {
      std::cout << "Collision detected\n";
      collision = true;
  }
  if(cvRes && !myRes)
  {
    std::cout << "Collision detected in cv check but not mine\n";
  }
  else if (myRes && !cvRes)
  {
    std::cout << "Collision detected in my check but not cv\n";
  }
  
  if(collision)  // This will be replaced by a call to my custom 'isLessThan' function
  {
    double min, max;
    cv::Mat mask =  model_depth > 0;
    cv::minMaxIdx(model_depth, &min, &max, 0, 0, mask);
    std::cout << "Min: " << min << ", max: " << max << "\n";

    
    double alpha = 30;
    double scale = (255 - alpha) / (max - min);
    double delta = alpha - min *(255- alpha)/(max - min);
    
    cv::Mat viz;
    cv::convertScaleAbs(model_depth, viz, 255 / (max-min), -min);
    
    //viz = (model_depth - min) * (255 - alpha)/(max - min) + alpha;
    
    //cv::normalize(model_depth, model_depth, 0, 255, cv::NORM_MINMAX); //I thought this would do the same thing as the above, but I got a memory corruption error
    
    cv::imshow("model_depth", viz);
    cv::imshow("depth_diff", diff);
    cv::waitKey(200);
    return true;
  }
  return false;
}

cv::Mat DenseModel::generateHallucinatedRobotImpl(const geometry_msgs::Pose pose)
{
  if(isReady())
  {
    geometry_msgs::Pose adj_pose = pose;
    adj_pose.position.z = adj_pose.position.z + .15;  //raising model a little
    
    const geometry_msgs::Pose::ConstPtr pose_ptr(new geometry_msgs::Pose(pose));  // Note: if depth_model_ took in normal pointers to pose, etc, I could just pass in the address rather than creating a new object. Is there any reason not to do that?
    const sensor_msgs::Image::ConstPtr img_msg = depth_model_->generateDepthModel(pose_ptr, cv_image_ref_, cam_model_->cameraInfo());
    
    //Should add subscription checks
    pub_model_depth_image_.publish(*img_msg, cam_model_->cameraInfo());  // Should be able to publish ConstPtr messages!
    
    if(img_msg)
    {
      cv_bridge::CvImage::ConstPtr img_cv = cv_bridge::toCvShare(img_msg);
      //cv::imshow("depth_model", img_cv->image);
      //cv::waitKey(0);
      
      return img_cv->image;
    }
    else
    {
      ROS_WARN_NAMED(name_, "Warning: no depth image generated");
    }
  }
    
  return cv::Mat();
}

void DenseModel::setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im)
{

}

geometry_msgs::Pose DenseModel::transformPose(const geometry_msgs::Pose& pose)
{
  return pose;
}



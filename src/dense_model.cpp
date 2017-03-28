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
#include "mesh_filter/depth_model_tester.h"

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
  depth_model_ = std::make_shared<DepthModel>(tfBuffer_);

}

DenseModel::~DenseModel()
{
}

bool DenseModel::init()
{
  depth_model_->init("robot_description");
  

    
  input_depth_transport_ = std::make_shared<image_transport::ImageTransport>(nh_); //Note: really just need the camera info
  model_depth_transport_ = std::make_shared<image_transport::ImageTransport>(nh_);
  
  pub_model_depth_image_ = model_depth_transport_->advertiseCamera("model_depth", 1);
  sub_depth_image_ = input_depth_transport_->subscribeCamera("/camera/depth/image_raw", 5,
                                                            &DenseModel::depthCb, this); //hints
 
  pose_sub_ = nh_.subscribe("/pose", 1, &DenseModel::poseCB, this);



}




void DenseModel::depthCb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  ROS_DEBUG("depth callback");
  
  currentCameraInfo_ = std::make_shared<CameraSpecs>(depth_msg, info_msg);
  
}

bool DenseModel::isReady()
{
    
  if(!currentCameraInfo_)
  {
    ROS_WARN("Cannot render depth image until camera info is received!");
    return false;
  }
  
  if(!depth_model_->isReady())
  {
    return false;
  }
}

void DenseModel::poseCB(const geometry_msgs::PoseConstPtr& pose_msg)
{
  ROS_INFO("pose callback!");
  
  if(isReady())
  {
    std::shared_ptr<CameraSpecs> info = currentCameraInfo_;  //to prevent changes mid function

    const sensor_msgs::ImageConstPtr& depth_msg = info->depth_msg;
    const sensor_msgs::CameraInfoConstPtr& info_msg = info->info_msg;
    
    const sensor_msgs::ImageConstPtr debug_msg = depth_model_->generateDepthModel(pose_msg, depth_msg, info_msg);
    
    if(debug_msg)
    {
      pub_model_depth_image_.publish(*debug_msg, *info_msg);
    }
    
  }
    
}

bool DenseModel::testCollision(const geometry_msgs::Pose& pose)
{
  cv::Mat model_depth = generateHallucinatedRobot(pose);
  
}

cv::Mat DenseModel::generateHallucinatedRobot(float [] pt)
{
  if(isReady())
  {
    geometry_msgs::Pose pose;
    pose.position.x = pt[0];
    pose.position.y = pt[1];
    pose.position.z = pt[2];
    
    if(sizeof(pt) / sizeof(float) == 4)
    {
      pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pt[3]);
    }
    

  
  
}

cv::Mat DenseModel::generateHallucinatedRobot(const geometry_msgs::Pose& pose)
{

  
  
    

    const sensor_msgs::CameraInfoConstPtr info_msg(cam_model_->cameraInfo());
    
    const sensor_msgs::ImageConstPtr debug_msg = depth_model_->generateDepthModel(pose_msg, image_bridge_, info_msg);
    


}







int main(int argc, char **argv)
{
    std::string name= "depth_model";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    depth_projection::DenseModel model(nh, pnh);
    model.init();


    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();


	return 0;
}





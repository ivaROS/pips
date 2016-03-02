#include <ros/ros.h>
#include <iostream>     // std::cout
#include <algorithm>    // std::min

#include <tf2_ros/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>


#include <sensor_msgs/image_encodings.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <climits>

#include "collision_checker.h"

#define PUBLISH_DEPTH_IMAGE false   //When true, publishes the input depth image with projected robot
#define MAX_RANGE 10    //Maximum expected range of sensor, used to fill unknowns (0's) in image
#define DEBUG false 
#define SCALE_METERS 1
#define SCALE_MM 1000


  //Construct
  CollisionChecker::CollisionChecker(geometry_msgs::TransformStamped& base_optical_transform, std::vector<cv::Point3d> co_points, bool pub_image)
    :it_(nh_), co_offsets_(co_points), publish_image_(pub_image)
  {

    if(publish_image_)
    {
      //Create publisher with large queue due to sheer number of images produced
      depthpub_ = it_.advertise("depth_image_out",2000);
    }
    
    
    optical_transform_ = tf2::transformToEigen(base_optical_transform);

    if(DEBUG)
      std::cout << "construct collision checker" << std::endl;

  }


  void CollisionChecker::setImage(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if(DEBUG)
    std::cout << "[collision_checker] Setting new image" << std::endl;

    total_duration = std::chrono::duration<double, std::milli>::zero(); //reset accumulated time
    
    try {

      
      cv_bridge::CvImageConstPtr input_bridge_ref;
      input_bridge_ref = cv_bridge::toCvCopy(image_msg);
      
      //If data type 32bit float, unit is m; else mm
      scale_ = (input_bridge_ref->encoding == sensor_msgs::image_encodings::TYPE_32FC1) ? SCALE_METERS : SCALE_MM;
      
      //Make a copy of depth image
      image_ref_ = input_bridge_ref->image;
      
      //Set all 0's in image to some large value
      image_ref_.setTo(MAX_RANGE * scale_, image_ref_==0);

      if(publish_image_)
      {
        //Make extra copy of depth image for publishing purposes
        input_bridge_ = cv_bridge::toCvCopy(image_msg,  sensor_msgs::image_encodings::TYPE_32FC1); 
        (input_bridge_->image).copyTo(image_);
      }
      
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }
    
    //Reinitialize camera model with each image in case resolution has changed
    cam_model_.fromCameraInfo(info_msg);

  }

      

  //coordinates [x,y,z] are in the robot base's coordinate frame.
  bool CollisionChecker::testCollision(double xyz[])
  {
    //Convert coordinates to Eigen Vector
    Eigen::Map<const Eigen::Vector3d> origin_r(xyz);
    
    //Start the clock
    auto t1 = std::chrono::high_resolution_clock::now();


    cv::Mat image;
    if(publish_image_)
    {  
      //Make local copy of image so we can draw on it
      image = input_bridge_->image;
      image_.copyTo(image);
    }

    //Transform coordinates of robot base into camera's optical frame
    Eigen::Vector3d origin_d = optical_transform_*origin_r;
    //std::cout << "point (optical): " << origin_d << std::endl;

    //Convert datatype of coordinates
    cv::Point3d pt_cv(origin_d(0), origin_d(1), origin_d(2));

    
    cv::Point2d co_uv[4];
    double co_depth;

    for (int it = 0; it <4; ++it) {
      //Add specified offsets to robot's base position
      cv::Point3d addedpnt = pt_cv + co_offsets_[it];
      
      //Store the depth of the collision object
      co_depth = addedpnt.z;
      
      //Project point to pixel coordinates
      cv::Point2d uv = cam_model_.project3dToPixel(addedpnt);
      //ROS_DEBUG("Coords: %f, %f", uv.x, uv.y);
      co_uv[it] = uv;

    }
    

    //Using the 4 points, construct a rectangle
    double minXVal, maxXVal, minYVal,maxYVal;
    minYVal = std::min(image_ref_.rows-1.0,std::max(0.0,std::min(co_uv[0].y,co_uv[1].y)));
    minXVal = std::min(image_ref_.cols-1.0,std::max(0.0, std::min(co_uv[1].x,co_uv[2].x)));
    maxYVal = std::max(0.0, std::min(image_ref_.rows-1.0, std::max(co_uv[2].y,co_uv[3].y)));
    maxXVal = std::max(0.0, std::min(image_ref_.cols-1.0, std::max(co_uv[3].x,co_uv[0].x)));

    cv::Point2d topL(minXVal, minYVal);
    cv::Point2d bottomR(maxXVal, maxYVal);

    cv::Rect co_rect(topL, bottomR);

    //The following should be a more elegant way to take the collision outline and crop it to fit in the image
    //cv::Rect co_rect = Rect(topL, bottomR) &= Rect(Point(0, 0), image_ref_->size());

    //The collision object rectangle is our ROI in the original image
    cv::Mat roi(image_ref_,co_rect);
    
    //Check if any points in ROI are closer than collision object's depth
    cv::Mat collisions = (roi <= co_depth*scale_);
    int num_collisions = cv::countNonZero(collisions);
    
    bool collided = (num_collisions>0);
    
    //Calculate elapsed time for this computation and update total duration
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
    total_duration += fp_ms;
    
    
    if(DEBUG)
    ROS_INFO_STREAM("Collision checking took " << fp_ms.count() << " ms; accumulated time: " << total_duration.count() << " ms\n");

    if(collided)
    if(DEBUG)
        ROS_INFO_STREAM("Collided! (" << num_collisions << ")\n");

    if(publish_image_)
    {
      //Draw collision object on depth image and publish it
      collisions.convertTo(collisions,CV_32F,1000);
      cv::rectangle(image, co_rect, co_depth*scale_, CV_FILLED);
      depthpub_.publish(input_bridge_->toImageMsg());

    }
    
    return collided;
  }




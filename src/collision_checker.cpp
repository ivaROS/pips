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

#define PUBLISH_DEPTH_IMAGE false
#define DRAW_DEPTH_POINTS false
#define MAX_RANGE 10

  CollisionChecker::CollisionChecker(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg, geometry_msgs::TransformStamped& base_optical_transform, std::vector<cv::Point3d> co_points, bool gen_image)
    :it_(nh_), co_offsets_(co_points), generate_image_(gen_image)
  {
    scale_ = 1000;
    
    if(PUBLISH_DEPTH_IMAGE)
      depthpub_ = it_.advertise("depth_image_out",1);
    
    base_to_optical_transform_ = tf2::transformToEigen(base_optical_transform);
    optical_transform_ = base_to_optical_transform_;

    //tf::transformStampedMsgToTF(const geometry_msgs::TransformStamped & msg, TransformStamped& bt)

    std::cout << "init collision checker" << std::endl;


    try {
      if(PUBLISH_DEPTH_IMAGE)
      {
        input_bridge_ = cv_bridge::toCvCopy(image_msg,  sensor_msgs::image_encodings::TYPE_32FC1); 
        (input_bridge_->image).copyTo(image_);
      }
      
      cv_bridge::CvImageConstPtr input_bridge_ref;
      input_bridge_ref = cv_bridge::toCvCopy(image_msg);//Note:since only comparing and not editing image, no need to copy data
      //However, would it be faster to copy the data so I could get rid of 0s and only perform 1 comparision on the image /run rather than 2?
      
      image_ref_ = input_bridge_ref->image;
      
      //double min, max;
      //cv::minMaxLoc(image_ref_, &min, &max);
      
      //std::cout << "Min: " << min << ", Max: " << max << std::endl;

            //is it 32bit float? Then unit is m
      if(input_bridge_ref->encoding == sensor_msgs::image_encodings::TYPE_32FC1) 
      {
          scale_ = 1;
      }
      
            
      image_ref_.setTo(MAX_RANGE * scale_, image_ref_==0);

      //std::cout << "altered mat:\n" << image_ref_ << std::endl;

      
      /*
      //is it 16bit unsigned int? Then unit is mm
      else if(input_bridge_ref->encoding == sensor_msgs::image_encodings::TYPE_16UC1) 
      {
        scale_ = 1000;
      }
      //std::cout << "ref type: " << input_bridge_ref->encoding<< ", im type: " << input_bridge->encoding << std::endl;
      */
      
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }
    
    cam_model_.fromCameraInfo(info_msg);

  }


  // Represents the transform that will project points from the robot's frame to the global frame
  //(generally odom).
  void CollisionChecker::setBaseTransform(geometry_msgs::TransformStamped& base_transform)
  {
       Eigen::Affine3d coord_to_base_transform = tf2::transformToEigen(base_transform);
       coord_to_base_transform = coord_to_base_transform.inverse();
       
       optical_transform_ = base_to_optical_transform_ * coord_to_base_transform;
  }


  // By default, coordinates [x,y,z] are considered to be in the robot base's coordinate frame.
  // Some other frame can be set as default by calling setBaseTransform with the transfrom from
  // the robot's frame to the other frame
  bool CollisionChecker::testCollision(double xyz[])
  {
      //std::cout << "point (start) x: " << xyz[0] << ", y: " << xyz[1] << ", z: " << xyz[2] << std::endl;
      Eigen::Map<const Eigen::Vector3d> origin_r(xyz);

      auto t1 = std::chrono::high_resolution_clock::now();
    
    
      cv::Mat image;
      if(PUBLISH_DEPTH_IMAGE)
      {  
         image = input_bridge_->image;
         image_.copyTo(image);
      }

      
      Eigen::Vector3d origin_d = optical_transform_*origin_r;
      //std::cout << "point (optical): " << origin_d << std::endl;

      cv::Point3d pt_cv(origin_d(0), origin_d(1), origin_d(2));
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);
      
      //std::cout << "point (camera): " << uv << std::endl;

      static const int RADIUS = 3;
      if(PUBLISH_DEPTH_IMAGE and DRAW_DEPTH_POINTS)
            cv::circle(image, uv, RADIUS, (2^16)-1, -1);

      cv::Point2d co_uv[4];
      double co_depth;

      for (int it = 0; it <4; ++it) {
      
        cv::Point3d addedpnt = pt_cv + co_offsets_[it];
        co_depth = addedpnt.z;
        uv = cam_model_.project3dToPixel(addedpnt);
        //ROS_DEBUG("Coords: %f, %f", uv.x, uv.y);
        co_uv[it] = uv;

      if(PUBLISH_DEPTH_IMAGE and DRAW_DEPTH_POINTS)
        cv::circle(image, uv, RADIUS, (2^16)-1, -1);
      }
      

      
      double minXVal, maxXVal, minYVal,maxYVal;
      minYVal = std::min(image_ref_.rows-1.0,std::max(0.0,std::min(co_uv[0].y,co_uv[1].y)));
      minXVal = std::min(image_ref_.cols-1.0,std::max(0.0, std::min(co_uv[1].x,co_uv[2].x)));
      maxYVal = std::max(0.0, std::min(image_ref_.rows-1.0, std::max(co_uv[2].y,co_uv[3].y)));
      maxXVal = std::max(0.0, std::min(image_ref_.cols-1.0, std::max(co_uv[3].x,co_uv[0].x)));
   
      cv::Point2d topL(minXVal, minYVal);
      cv::Point2d bottomR(maxXVal, maxYVal);

      //ROS_DEBUG("Raw Rectangle: %f, %f and %f, %f", co_uv.at(1).x, co_uv.at(1).y, co_uv.at(3).x, co_uv.at(3).y);
      //ROS_DEBUG("Cropped Rectangle: %f, %f and %f, %f", topL.x, topL.y, bottomR.x, bottomR.y);
     
      cv::Rect co_rect(topL, bottomR);

      //The following should be a more elegant way to take the collision outline and crop it to fit in the image
      //cv::Rect co_rect = Rect(topL, bottomR) &= Rect(Point(0, 0), image_ref_->size());

      cv::Mat roi(image_ref_,co_rect);
      
      
      cv::Mat collisions = (roi <= co_depth*scale_);
      int num_collisions = cv::countNonZero(collisions);
      
      bool collided = (num_collisions>0);
      
      //Calculate elapsed time
      auto t2 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
      total_duration += fp_ms;
      
      std::cout << "Collision checking took " << fp_ms.count() << " ms; accumulated time: " << total_duration.count() << " ms\n";

      if(collided)
        std::cout << "Collided! (" << num_collisions << ")\n";

      if(PUBLISH_DEPTH_IMAGE)
      {
        collisions.convertTo(collisions,CV_32F,1000);
        cv::rectangle(image, co_rect, co_depth*scale_, CV_FILLED);
        depthpub_.publish(input_bridge_->toImageMsg());

      }
      
      return collided;
   
  }




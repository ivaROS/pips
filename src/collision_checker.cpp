#include "collision_checker.h"

#include "hallucinated_robot_model.h"

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




#define MAX_RANGE 10    //Maximum expected range of sensor, used to fill unknowns (0's) in image
#define SCALE_METERS 1
#define SCALE_MM 1000


  /*
  Description: The CollisionChecker class is responsible for determining whether a future robot pose will collide with any obstacles. Intended usage: 
    Construct a single instance
    Whenever new image arrives, call setImage()
    For each point in a trajectory, call testCollision()
  */
  
  
  
  /*
  Description: The constructor takes in only arguments that are unlikely to change, therefore it only needs to be called once.
  Name: CollisionChecker::CollisionChecker
  Inputs:
    base_optical_transform: The transform from the base coordinate frame to the depth sensor's optical frame.
    robot_model: The object that defines how the hallucinated robot is represented and that performs the actual collision comparison.
    pub_image: When enabled, the collision object's projection is added to the current depth image and published. Only enable when debugging- during normal usage far too many images would be published.
  */ 
  
  /* The publish_image_ parameter will either be moved to dyanmic reconfigure or will be automatically toggled 
     based on the presence of subscribers
  */
  CollisionChecker::CollisionChecker(geometry_msgs::TransformStamped& base_optical_transform) :
    it_(nh_), publish_image_(false), robot_model_(nh_)
  {
    if(publish_image_)
      {
        //Create publisher with large queue due to sheer number of images produced
        depthpub_ = it_.advertise("depth_image_out",2000);
      }
      
      //Convert transform to Eigen
      optical_transform_ = tf2::transformToEigen(base_optical_transform);

      depth_generation_service_ = nh_.advertiseService("generate_depth_image", &CollisionChecker::getDepthImage, this);

      ROS_DEBUG_STREAM("[collision_checker] Constructing collision checker");

   
  
  }
  
  
  
  /*
  Description: Sets the CollisionChecker's depth image and camera model. Called whenever there is a new image.
  Name: CollisionChecker::setImage
  Inputs:
    image_msg: The depth image. Either the image_raw (16bit unsigned) or image (32 bit float) topics may be used, but the raw topic is preferred.
    info_msgs: The CameraInfo msg accompanying the image msg. It is necessary to update the camera model each time in order to permit changing the camera's resolution during operation.
  */ 
  void CollisionChecker::setImage(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    ROS_DEBUG_STREAM("[collision_checker] Setting new image" << std::endl);
    
    
    try {
      
      cv_bridge::CvImagePtr input_bridge_ref;
      input_bridge_ref = cv_bridge::toCvCopy(image_msg);
      
      //If data type 32bit float, unit is m; else mm
      scale_ = (input_bridge_ref->encoding == sensor_msgs::image_encodings::TYPE_32FC1) ? SCALE_METERS : SCALE_MM;
      
      //Make a copy of depth image and set all 0's (unknowns) in image to some large value
      image_ref_ = input_bridge_ref->image;
      image_ref_.setTo(MAX_RANGE * scale_, image_ref_==0);

      //A gazebo difference: unknown values are given as 'nan' rather than 0. However, comparison of a Nan with a number will always return false, so it won't trigger a collision

      if(publish_image_)
      {
        //Make extra copy of depth image for publishing purposes
        input_bridge_ = cv_bridge::toCvCopy(image_msg,  sensor_msgs::image_encodings::TYPE_32FC1); 
        (input_bridge_->image).copyTo(image_);
      }
      
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[collision_checker] Failed to convert image");
      return;
    }
/*    catch (cv::Exception& ex){
      ROS_ERROR("[collision_checker] OpenCV Exception");
      return;
    }
  */  
    //Reinitialize camera model with each image in case resolution has changed

    robot_model_.updateModel(image_ref_, info_msg, scale_);

  }

      
  /*
  Description: Tests if the specified base coordinates would result in a collision. Global variables are not modified, allowing this method to be called from multiple threads simultaneously
  Name: CollisionChecker::testCollision
  Inputs:
    xyz: test coordinates [x,y,z] in the robot base's coordinate frame.
  Output:
    bool: coordinates cause collision
  */ 
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

    //Check if any points in ROI are closer than collision object's depth
/*
    cv::Mat collisions = (roi <= co_depth*scale_);
    int num_collisions = cv::countNonZero(collisions);
    bool collided = (num_collisions>0);
  */
  


    
    
    bool collided = robot_model_.testCollision(pt_cv);
    
    //Calculate elapsed time for this computation
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
    
    
    ROS_DEBUG_STREAM("[collision_checker] Collision checking took " << fp_ms.count() << " ms");

   // ROS_DEBUG_STREAM_COND(collided, "[collision_checker] Collided! (" << num_collisions << ")");
    //ROS_DEBUG_STREAM_COND(collided, "[collision_checker] Collided! Nearest world point: " << min_depth);


    if(publish_image_ && depthpub_.getNumSubscribers() > 0)
    {
      //Draw collision object on depth image and publish it
      //collisions.convertTo(collisions,CV_32F,1000);
      //cv::rectangle(image, co_rect, co_depth*scale_, CV_FILLED);
      depthpub_.publish(input_bridge_->toImageMsg());

    }
  
    return collided;
  }
  
  cv::Mat CollisionChecker::generateDepthImage(double xyz[])
  {
    //Convert coordinates to Eigen Vector
    Eigen::Map<const Eigen::Vector3d> origin_r(xyz);
    
    //Transform coordinates of robot base into camera's optical frame
    Eigen::Vector3d origin_d = optical_transform_*origin_r;
    //std::cout << "point (optical): " << origin_d << std::endl;

    //Convert datatype of coordinates
    cv::Point3d pt_cv(origin_d(0), origin_d(1), origin_d(2));

    return robot_model_.generateHallucinatedRobot(pt_cv);
    
  }
  
  bool CollisionChecker::getDepthImage(pips::GenerateDepthImage::Request &req, pips::GenerateDepthImage::Response &res)
  {
    ROS_INFO_STREAM("Depth image generation request received.");
    
    double coords[3];
    coords[0] = req.pose.position.x;
    coords[1] = req.pose.position.y;
    coords[2] = req.pose.position.z;
    
    cv_bridge::CvImage out_msg;
    //out_msg.header   = ; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;// (scale_ == SCALE_METERS) ? sensor_msgs::image_encodings::TYPE_32FC1 : sensor_msgs::image_encodings::TYPE_16UC1;
    out_msg.image = generateDepthImage(coords);
    
    res.image = *out_msg.toImageMsg();

    
    return true;
  }

  // Incomplete and Not currently used
  void CollisionChecker::generateImageCoord(const double xyz[], double * uv)
  {
    //Convert coordinates to Eigen Vector
    Eigen::Map<const Eigen::Vector3d> origin_r(xyz);
    
    //Transform coordinates of robot base into camera's optical frame
    Eigen::Vector3d origin_d = optical_transform_*origin_r;
    //std::cout << "point (optical): " << origin_d << std::endl;

    // convert into image coordinate

    
  }




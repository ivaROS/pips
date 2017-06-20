#include "pips/collision_testing/pips_collision_checker.h"

#include "pips/collision_testing/robot_models/hallucinated_robot_model.h"

#include <pips/GenerateDepthImage.h>
#include <pips/TestCollision.h>

#include <cv_bridge/cv_bridge.h>

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
  Description: The PipsCollisionChecker class is responsible for determining whether a future robot pose will collide with any obstacles. Intended usage: 
    Construct a single instance
    Whenever new image arrives, call setImage()
    For each point in a trajectory, call testCollision()
  */
  
  
  
  /*
  Description: The constructor takes in only arguments that are unlikely to change, therefore it only needs to be called once.
  Name: PipsCollisionChecker::PipsCollisionChecker
  Inputs:
    base_optical_transform: The transform from the base coordinate frame to the depth sensor's optical frame.
    robot_model: The object that defines how the hallucinated robot is represented and that performs the actual collision comparison.
    pub_image: When enabled, the collision object's projection is added to the current depth image and published. Only enable when debugging- during normal usage far too many images would be published.
  */ 
  
  /* The publish_image_ parameter will either be moved to dyanmic reconfigure or will be automatically toggled 
     based on the presence of subscribers
  */
  PipsCollisionChecker::PipsCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh) : CollisionChecker(nh,pnh),
    nh_(nh, name_), pnh_(pnh, name_), it_(nh_), robot_model_(nh_, pnh_)
  {
      ROS_DEBUG_STREAM_NAMED(name_, "Constructing collision checker");
  }

  void PipsCollisionChecker::initImpl()
  {
      robot_model_.init();
      depth_generation_service_ = nh_.advertiseService("generate_depth_image", &PipsCollisionChecker::getDepthImageSrv, this);
  }
  
    void PipsCollisionChecker::setTransform(const geometry_msgs::TransformStamped& base_optical_transform)
    {
      //Convert transform to Eigen
      optical_transform_ = tf2::transformToEigen(base_optical_transform);
      base_optical_transform_ = base_optical_transform;
      robot_model_.setTransform(base_optical_transform);
    }
  
  /*
  Description: Sets the PipsCollisionChecker's depth image and camera model. Called whenever there is a new image.
  Name: PipsCollisionChecker::setImage
  Inputs:
    image_msg: The depth image. Either the image_raw (16bit unsigned) or image (32 bit float) topics may be used, but the raw topic is preferred (the reason being that the conversion nodelet is not needed and in theory we save some computation. However, the overhead is quite minimal, and it appears that some SIMD optimizations can only work with floats, so whatever).
    info_msgs: The CameraInfo msg accompanying the image msg. It is necessary to update the camera model each time in order to permit changing the camera's resolution during operation.
  */ 
  void PipsCollisionChecker::setImage(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Setting new image" << std::endl);
    
    try {
      
      input_bridge_ref_ = cv_bridge::toCvCopy(image_msg);
      
      //If data type 32bit float, unit is m; else mm
      scale_ = (input_bridge_ref_->encoding == sensor_msgs::image_encodings::TYPE_32FC1) ? SCALE_METERS : SCALE_MM;
      
      //Make a copy of depth image and set all 0's (unknowns) in image to some large value
      image_ref_ = input_bridge_ref_->image;
      image_ref_.setTo(MAX_RANGE * scale_, image_ref_==0);
      
      
      //image_ref_.setTo(MAX_RANGE * scale_, image_ref_!=image_ref_);

      //A gazebo difference: unknown values are given as 'nan' rather than 0. However, comparison of a Nan with a number will always return false, so it won't trigger a collision
      //That also means that a simulation-only version could skip the data copy and 0 replacement. Futhermore, the driver could be customized to use Nans too

    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR_NAMED(name_, "Failed to convert image");
      return;
    }
 
    //Reinitialize camera model with each image in case resolution has changed

    robot_model_.updateModel(input_bridge_ref_, info_msg, scale_);

  }

      
  /*
  Description: Tests if the specified base coordinates would result in a collision. Global variables are not modified, allowing this method to be called from multiple threads simultaneously
  Name: PipsCollisionChecker::testCollision
  Inputs:
    xyz: test coordinates [x,y,z] in the robot base's coordinate frame. Need to add \theta
  Output:
    bool: coordinates cause collision
  */ 
  bool PipsCollisionChecker::testCollisionImpl(geometry_msgs::Pose pose)
  {
    bool collided = robot_model_.testCollision(pose);
    
    return collided;
  }
  
  
  cv::Mat PipsCollisionChecker::generateDepthImage(geometry_msgs::Pose pose)
  {
    return robot_model_.generateHallucinatedRobot(pose);
    
  }
  
  bool PipsCollisionChecker::getDepthImageSrv(pips::GenerateDepthImage::Request &req, pips::GenerateDepthImage::Response &res)
  {
    ROS_INFO_STREAM_NAMED(name_, "Depth image generation request received.");
 
    cv_bridge::CvImage out_msg;
    //out_msg.header   = ; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;// (scale_ == SCALE_METERS) ? sensor_msgs::image_encodings::TYPE_32FC1 : sensor_msgs::image_encodings::TYPE_16UC1;
    out_msg.image = generateDepthImage(req.pose);
    
    res.image = *out_msg.toImageMsg();
    
    return true;
  }

  // Incomplete and Not currently used
  void PipsCollisionChecker::generateImageCoord(const double xyz[], double * uv)
  {
    //Convert coordinates to Eigen Vector
    Eigen::Map<const Eigen::Vector3d> origin_r(xyz);
    
    //Transform coordinates of robot base into camera's optical frame
    Eigen::Vector3d origin_d = optical_transform_*origin_r;
    //std::cout << "point (optical): " << origin_d << std::endl;

    // convert into image coordinate

    
  }
  
  
  
  
  /*
  PoseType getPose(geometry_msgs::Point point)
  {
    geometry_msgs::Pose pose;
    pose.position = point;
    
    return getPose(pose);
  }
  
  eigen::Affine3d getPose(geometry_msgs::Pose pose_out)
  {
    eigen::Affine3d pose_out;
    tf2::fromMsg(pose_in, pose_out);
  }
  
  */



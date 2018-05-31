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

#include <climits>


#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//#include <pcl/conversions.h>
//#include <pcl_ros/transforms.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


//#include <extended_local/ExtPose.h>
//#include <extended_local/ExtPoseRequest.h>
//#include <extended_local/ExtPoseResponse.h>


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
  
  
  

  PipsCollisionChecker::PipsCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name) : 
    pips::collision_testing::TransformingCollisionChecker(nh,pnh),
    name_(name),
    nh_(nh, name_),
    pnh_(pnh, name_),
    robot_model_(nh_, pnh_,""),
    setup_durations_(name_, "pips_construction")
  {
      ROS_DEBUG_STREAM_NAMED(name_, "Constructing collision checker");

  }

  void PipsCollisionChecker::initImpl()
  {
    cam_model_ = getCameraModel();
    
    robot_model_.setCameraModel(cam_model_);
    
      robot_model_.init();
      
      //checker_ = nh_.serviceClient<extended_local::ExtPose>("/ext_check");

      
      //TODO: This should probably accept a CameraInfo message as an optional parameter, allowing it to be used without a camera
      depth_generation_service_ = pnh_.advertiseService("generate_depth_image", &PipsCollisionChecker::getDepthImageSrv, this);
      posepub_ = pnh_.advertise<geometry_msgs::PoseStamped>("collision_poses",100);
      
      pointpub_ = pnh_.advertise<PointCloud>("collisions",100);

  }
  
  /*Currently, I don't use the 'optical_transform' anywhere. Everything happens within the robot model 
  * (this was to enable optimized transformations of different representations, which really wasn't worth implementing)
  */
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
    image_msg: The depth image. Either the image_raw (16bit unsigned) or image (32 bit float) topics may be used, but the raw topic is preferred (the reason being that the conversion nodelet is not needed and in theory we save some computation. More importantly, 16u types can be vectorized more than 32f).
    info_msgs: The CameraInfo msg accompanying the image msg. It is necessary to update the camera model each time in order to permit changing the camera's resolution during operation.
  */ 
  void PipsCollisionChecker::setImage(const sensor_msgs::ImageConstPtr& image_msg)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Setting new image" << std::endl);
    
    try {
      
      input_bridge_ref_ = cv_bridge::toCvCopy(image_msg);
      
      //If data type 32bit float, unit is m; else mm
      scale_ = (input_bridge_ref_->encoding == sensor_msgs::image_encodings::TYPE_32FC1) ? SCALE_METERS : SCALE_MM;
      
      //Make a copy of depth image and set all 0's (unknowns) in image to some large value.
      //TODO: use max value of type rather than max range of kinect
      //TODO: only perform this check if type is float
      image_ref_ = input_bridge_ref_->image;
      
      auto t1 = ros::WallTime::now();
      image_ref_.setTo(MAX_RANGE * scale_, image_ref_==0);
      auto t2 = ros::WallTime::now();
      
      int64_t duration = (t2-t1).toNSec();
      setup_durations_.addDuration(t1,t2);
    
      ROS_DEBUG_STREAM_NAMED("CollisionChecker.current_duration", "[CollisionChecker]: Setup Duration = " << duration << ", size=" << image_ref_.cols*image_ref_.rows);
      
      //image_ref_.setTo(MAX_RANGE * scale_, image_ref_!=image_ref_);

      //A gazebo difference: unknown values are given as 'nan' rather than 0. However, comparison of a Nan with a number will always return false, so it won't trigger a collision
      //That also means that a simulation-only version could skip the data copy and 0 replacement. Futhermore, the driver could be customized to use Nans too

    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR_NAMED(name_, "Failed to convert image");
      return;
    }

    //Reinitialize camera model with each image in case resolution has changed
    
    robot_model_.updateModel(input_bridge_ref_, scale_);
    return;
  }

      
  /*
  Description: Tests if the specified base coordinates would result in a collision. Global variables are not modified, allowing this method to be called from multiple threads simultaneously
  Name: PipsCollisionChecker::testCollision
  Inputs:
    xyz: test coordinates [x,y,z] in the robot base's coordinate frame. Need to add \theta
  Output:
    bool: coordinates cause collision
  */ 
  CCResult PipsCollisionChecker::testCollisionImpl(geometry_msgs::Pose pose, CCOptions options)
  {
    CCResult collided = robot_model_.testCollision(pose, options);
    
    if(collided)
    { 
      
	if(collided.details())
	{
	    std::vector<cv::Point3d> worldPoints = collided.getCollisionPnts();
	    
	    ROS_INFO_STREAM_NAMED(name_, "point depth: " << worldPoints[0].z);
	    	    
	    PointCloud::Ptr msg (new PointCloud);
	    msg->header.stamp = input_bridge_ref_->header.stamp.toNSec()/1e3; // ros::Time::now().toNSec()/1e3;	//https://answers.ros.org/question/172241/pcl-and-rostime/
	    msg->header.frame_id = input_bridge_ref_->header.frame_id;// "camera_depth_optical_frame";
	    msg->height = 1;
	    //msg->points.insert(std::end(msg->points), std::begin(worldPoints), std::end(worldPoints));
	    msg->width = worldPoints.size();
	    
	    for(auto point : worldPoints)
	    {
	      msg->points.push_back (pcl::PointXYZ(point.x, point.y, point.z));
	    }
	    
	    pointpub_.publish(msg);
	}
      
      
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.pose = pose;
      pose_stamped.header.stamp = input_bridge_ref_->header.stamp; //This is only threadsafe if it is known that collision checks will not happen at the same time as updating camera info... It might be best to switch from Pose to PoseStamped for everything...
      pose_stamped.header.frame_id = base_optical_transform_.child_frame_id;
      posepub_.publish(pose_stamped);
    }
   
    
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




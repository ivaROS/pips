#include <pips/collision_testing/pips_collision_checker.h>
#include <pips/collision_testing/image_geometry_models/image_geometry_converter.h>
#include <pips/utils/image_comparison_implementations.h>

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

#include <sensor_msgs/image_encodings.h>

#include <climits>
#include <limits>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//#include <pcl/conversions.h>
//#include <pcl_ros/transforms.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


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
  
  
  

  PipsCollisionChecker::PipsCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tfm) : 
    pips::collision_testing::GeneralCollisionChecker(nh,pnh,name,tfm)
  {
    ROS_DEBUG_STREAM_NAMED(name_+".constructor", "Constructing collision checker");
    transpose_=true;
    robot_model_ = std::make_shared<pips::collision_testing::robot_models::RobotModel>(nh, pnh, tfm);
  }
  
  PipsCollisionChecker::PipsCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name) : 
    pips::collision_testing::GeneralCollisionChecker(nh,pnh,name,tfm)
  {
    ROS_DEBUG_STREAM_NAMED(name_+".constructor", "Constructing collision checker");
    transpose_=true;
    robot_model_ = std::make_shared<pips::collision_testing::robot_models::RobotModel>(nh, pnh, tfm);
  }

  void PipsCollisionChecker::initImpl()
  {
    pips::collision_testing::GeneralCollisionChecker::initImpl();
    cam_model_ = getCameraModel();
    
    //TODO: This should probably accept a CameraInfo message as an optional parameter, allowing it to be used without a camera
    depth_generation_service_ = pnh_.advertiseService("generate_depth_image", &PipsCollisionChecker::getDepthImageSrv, this);
  }
  
  /*
  Description: Sets the PipsCollisionChecker's depth image and camera model. Called whenever there is a new image.
  Name: PipsCollisionChecker::setImage
  Inputs:
    image_msg: The depth image. Either the image_raw (16bit unsigned) or image (32 bit float) topics may be used, but the raw topic is preferred (the reason being that the conversion nodelet is not needed and in theory we save some computation. More importantly, 16u types can be vectorized more than 32f). *Vectorization amount depends on platform. With some versions of SSE, the pipeline width is twice as large for floats, effectively canceling any gains from using ints. On others, they are the same width, and ints win.
    info_msgs: The CameraInfo msg accompanying the image msg. It is necessary to update the camera model each time in order to permit changing the camera's resolution during operation.
  */ 
  void PipsCollisionChecker::setImage(const sensor_msgs::ImageConstPtr& image_msg)
  {
    ROS_DEBUG_STREAM_NAMED(name_+".setup", "Setting new image" << std::endl);
    
    auto t1 = ros::WallTime::now();

    boost::mutex::scoped_lock lock(img_mutex_);

    try {  

      if(image_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      {
        input_bridge_ref_ = cv_bridge::toCvShare(image_msg);
        image_ref_ = input_bridge_ref_->image;
        scale_ = SCALE_METERS;
      }
      else if(image_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {
        //Make a copy of depth image and set all 0's (unknowns) in image to some large value.
        cv_bridge::CvImagePtr cv_im = cv_bridge::toCvCopy(image_msg);
        scale_ = SCALE_MM;
        
        image_ref_ = cv_im->image;
        image_ref_.setTo(std::numeric_limits<uint16_t>::max(), image_ref_==0);
        
        input_bridge_ref_ = cv_im;
      }
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR_NAMED(name_+".setup", "Failed to convert image");
      return;
    }
    
    if(transpose_)
    {
      image_ref_ = image_ref_.t();
    }
    
    auto t2 = ros::WallTime::now();
    
    setup_durations_.addDuration(t1,t2);
    
    ROS_DEBUG_STREAM_NAMED(name_ + ".setup_timing", "[" + name_ + "]: Setup Duration = " << setup_durations_.getLastDuration() << ", size=" << input_bridge_ref_->image.cols*input_bridge_ref_->image.rows << ", average setup duration = " << setup_durations_.averageDuration() );
    
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
    ComparisonResult result = imageSpaceCollisionImpl(pose, options);
    
    if(options.get_details_ && result.collides() && result.hasDetails())
    {	
      std::vector<CollisionPoint> world_points;
      for(auto point : result.points())
      {
        cv::Point3d ray = cam_model_->projectPixelTo3dRay(point.pt);
        cv::Point3d worldPoint = ray * (point.depth / scale_);
        ROS_DEBUG_STREAM_NAMED(name_+".collision_points", "Collision pixel coordinates: (" << point.pt.x << "," << point.pt.y << "), point: " << worldPoint.x << "," << worldPoint.y << "," << worldPoint.z);
        
        world_points.push_back(pips::collision_testing::toCollisionPoint(worldPoint));
      }
      
      return CCResult(world_points);
    }
    
    return CCResult(result.collides());
  }
  
  
  cv::Rect PipsCollisionChecker::getColumnRect(const int x, const int y, const int width, const int height)
  {
    if(transpose_)
    {
      return cv::Rect(y,x,height,width);
    }
    return cv::Rect(x,y,width,height);
  }
  
  cv::Rect PipsCollisionChecker::getColumnRect(const cv::Rect& rect)
  {
    return getColumnRect(rect.tl().x,rect.tl().y,rect.width,rect.height);
  }
  
  cv::Mat PipsCollisionChecker::generateDepthImage(geometry_msgs::Pose pose)
  {
    cv::Mat viz;
    cv::Size img_size = getImageRefSize();
    if(std::numeric_limits<float>::has_quiet_NaN)
    {
      double dNaN = std::numeric_limits<float>::quiet_NaN();
      viz = cv::Mat(img_size.height, img_size.width, image_ref_.type(), cv::Scalar(dNaN));
    }
    else
    {
      viz = cv::Mat::zeros(img_size.height, img_size.width, image_ref_.type());
    }
    
    int img_width = input_bridge_ref_->image.cols;
    int img_height = input_bridge_ref_->image.rows;
    
    ROS_DEBUG_STREAM_NAMED(name_+".image_generation", "Parent image dimensions: [" << viz.cols << "x" << viz.rows << "], image_ref dimensions: [" << img_width << "x" << img_height << "]");
    
    const std_msgs::Header header = getCurrentHeader();
    
    
    ROS_DEBUG_STREAM_NAMED(name_+".image_generation","Pose: " << toString(pose));
        
    auto models = robot_model_->getModel<pips::collision_testing::image_geometry_models::ImageGeometryConverter>(pose);
    
    for(const auto& model : models)
    {
      std::vector<COLUMN_TYPE> cols = model->getColumns(cam_model_, img_width, img_height);
      
      for(unsigned int i = 0; i < cols.size(); ++i)
      {
        cv::Rect roi = getColumnRect(cols[i].rect);
        cv::Mat col = cv::Mat(viz, roi); 
        float depth = cols[i].depth * scale_;
        //col.setTo(depth);
        col = cv::max(col,depth);
      }
    }
        
    if(transpose_)
    {
      cv::Mat viz_t = viz.t();
      return viz_t;
    }
    
    return viz;
  }

  cv::Size PipsCollisionChecker::getImageRefSize()
  {
      return image_ref_.size();
  }
  
  ComparisonResult PipsCollisionChecker::imageSpaceCollisionImpl(const geometry_msgs::Pose pose, CCOptions options)
  {
    boost::mutex::scoped_lock lock(img_mutex_);

    int img_width = input_bridge_ref_->image.cols;
    int img_height = input_bridge_ref_->image.rows;
    
    const std_msgs::Header header = getCurrentHeader();
    
    ROS_DEBUG_STREAM_NAMED(name_+".collision_test","Pose: " << toString(pose));
    
    auto models = robot_model_->getModel<pips::collision_testing::image_geometry_models::ImageGeometryConverter>(pose);
    
    ComparisonResult result;
    for(const auto& model : models)
    {      
      std::vector<COLUMN_TYPE> cols = model->getColumns(cam_model_, img_width, img_height);
      
      for(unsigned int i = 0; i < cols.size(); ++i)
      {
        //TODO: limit rect to image size
        cv::Rect roi = getColumnRect(cols[i].rect);
        cv::Mat col = cv::Mat(this->image_ref_,roi);
        float depth = std::max(cols[i].depth * scale_, (float)(0));
        
        ComparisonResult column_result = isLessThan(col, depth, options);
        
        if(column_result)
        {
          if(options.get_details_ && !column_result.hasDetails())
          {
            column_result = isLessThanDetails(col, depth, options);
          }
          cv::Point offset;
          cv::Size size;
          col.locateROI(size, offset);
          
          column_result.addOffset(offset);	
          
          if(options.full_details && column_result.hasDetails())
          {
            result.addResult(column_result);
          }
          else
          {
            result= column_result;
            goto done;
          }
        }
      }
    }
    
    done:
    if(transpose_)
    {
      result.transpose();
    }
      
    return result;
  }
  
  std_msgs::Header PipsCollisionChecker::getCurrentHeader()
  {
    if(input_bridge_ref_)
    {
      return input_bridge_ref_->header;
    }
    else
    {
      return std_msgs::Header();
    }
  }
  
  ComparisonResult PipsCollisionChecker::isLessThan(const cv::Mat& col, float depth, CCOptions options)
  {    
    return ::utils::isLessThan::vectorized(col, depth);
  }
  
  ComparisonResult PipsCollisionChecker::isLessThanDetails(const cv::Mat& col, float depth, CCOptions options)
  {
    // TODO: replace 'show_im_' with more accurate variable name (ex: 'full_details' or something)
    if(options.full_details)
    {
      ROS_DEBUG_STREAM_NAMED(name_+".details","FULL details!");
      return ::utils::isLessThan::fulldetails(col, depth);
    }
    else
    {
      return ::utils::isLessThan::details(col, depth);
    }
  }
  
  
  bool PipsCollisionChecker::getDepthImageSrv(pips::GenerateDepthImage::Request &req, pips::GenerateDepthImage::Response &res)
  {
    ROS_INFO_STREAM_NAMED(name_+".image_generation_service", "Depth image generation request received.");

    cv_bridge::CvImage out_msg;
    //out_msg.header   = ; // Same timestamp and tf frame as input image
    out_msg.encoding = input_bridge_ref_->encoding;
    out_msg.image = generateDepthImage(req.pose);
    
    res.image = *out_msg.toImageMsg();
    res.image.header = input_bridge_ref_->header;
    
    ROS_INFO_STREAM_NAMED(name_+".image_generation_service", "Image dimensions: [" << res.image.width << "x" << res.image.height << "], data size = " << sizeof(res.image.data) << ", num_elements: " << res.image.data.size());    
    return true;
  }




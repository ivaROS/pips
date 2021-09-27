
#include <pips/collision_testing/robot_models/box_model.h>
#include <pips/utils/image_comparison_implementations.h>

//#include <sensor_msgs/Image.h>
//#include <geometry_msgs/TransformStamped.h>

//#include <opencv/cv.h>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//#include <Eigen/Eigen>
//#include <image_transport/image_transport.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <cv_bridge/cv_bridge.h>

//#include <iomanip>      // std::setprecision

#include "ros/ros.h" //Needed for 'ROS_BREAK and ROS_ASSERT

    BoxModel::BoxModel() : HallucinatedRobotModelImpl<geometry_msgs::Pose>() 
    {
      this->name_ = "BoxModel";
    }
    
//TODO this goes back to HallucinatedRobotModelBase for paramete setting
  void BoxModel::setParameters(double width, double height, double length, double rear_distance, double safety_expansion, double floor_tolerance, bool show_im)
  {   
      robot_radius_ = width + safety_expansion;
      robot_height_ = height-floor_tolerance;
      robot_length = length + safety_expansion;
      floor_tolerance_ = floor_tolerance;
      distance_from_rear = rear_distance;
      
      show_im_ = show_im;
      
      geometry_msgs::TransformStamped origin;
      origin.transform.translation.z = robot_height_/2 + floor_tolerance;
      origin.transform.rotation.w = 1;
      
      box_ = pips::collision_testing::geometry_models::Box(robot_length, robot_radius_, robot_height_);
      box_.cam_model_ = cam_model_;
      box_.setOrigin(origin);
      tf2::doTransform(box_.origin_transform_, box_.current_transform_, base_optical_transform_); //TODO: don't make so many assumptions
      
  }
  
    void BoxModel::setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im)
    {
        //TODO Hard codding is the option used now(the two double variable)
        //Pioneer dimentions: length = 455, width = 381, height = 237. all dimentions in mm.
        double robo_length = 0.54;
        double dist_from_rare  = robo_length/2; // for now, mid way
        setParameters(radius, height, robo_length, dist_from_rare, safety_expansion, floor_tolerance, show_im);
    }

    //Don't transform orientation
    geometry_msgs::Pose BoxModel::transformPose(const geometry_msgs::Pose& pose)
    {
      Eigen::Affine3d pose_eig;
      tf2::fromMsg(pose, pose_eig);
      
      //Transform coordinates of robot base into camera's optical frame
      Eigen::Affine3d pose_eig_t;
      
      tf2::doTransform(pose_eig, pose_eig_t, box_.current_transform_);
      
      geometry_msgs::Pose pose_t;
      convertPose(pose_eig_t, pose_t);
      
      ROS_DEBUG_STREAM_NAMED(name_, "Pose " << toString(pose) << " transformed to " << toString(pose_t) );
      
      pose_t.orientation = pose.orientation;
      
      //ROS_DEBUG_STREAM_NAMED(name_, "Pose " << toString(pose) << " transformed to " << toString(pose_t) );
      
      return pose_t;
    }

  cv::Mat BoxModel::generateHallucinatedRobotImpl(const geometry_msgs::Pose pt)
  {
    cv::Mat viz = HallucinatedRobotModelImpl::generateHallucinatedRobotImpl(pt);
    
    std::vector<COLUMN_TYPE> cols = getColumns(pt);
    
    for(unsigned int i = 0; i < cols.size(); ++i)
    {
      cv::Mat col = cv::Mat(viz, cols[i].rect); //cols[i].image;
      float depth = cols[i].depth;
      
      col.setTo(depth);
    }
    
    return viz;
  }


  
  std::vector<COLUMN_TYPE> BoxModel::getColumns(const geometry_msgs::Pose pose)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Get columns for " << pose);
    return box_.getColumns(pose, cv_image_ref_->image.cols, cv_image_ref_->image.rows);
  }
    
      
  ComparisonResult BoxModel::testCollisionImpl(const geometry_msgs::Pose pose, CCOptions options)
  {
    std::vector<COLUMN_TYPE> cols = getColumns(pose);
  
    ComparisonResult result;
    for(unsigned int i = 0; i < cols.size(); ++i)
    {
      cv::Mat col = cv::Mat(this->image_ref_,cols[i].rect); //cols[i].image;
      float depth = cols[i].depth;
      
      ComparisonResult column_result = isLessThan(col, depth);

      if(column_result && options)
      {        
        if(!column_result.hasDetails())
        {
            column_result = isLessThanDetails(col,depth);
        }
        cv::Point offset;
        cv::Size size;
        col.locateROI(size, offset);
            
        column_result.addOffset(offset);	

        if(show_im_)
        {
          result.addResult(column_result);
        }
        else
        {
          return column_result;
        }
      }
    }
    return result;
  }
  

  ComparisonResult BoxModel::isLessThan(const cv::Mat& col, float depth)
  {
    return utils::isLessThan::stock(col, depth);
  }
  
  ComparisonResult BoxModel::isLessThanDetails(const cv::Mat& col, float depth)
  {
    // TODO: replace 'show_im_' with more accurate variable name (ex: 'full_details' or something)
    if(show_im_)
    {
      ROS_INFO_STREAM("FULL details!");
      return utils::isLessThan::fulldetails(col, depth);
    }
    else
    {
      return utils::isLessThan::details(col, depth);
    }
  }
    



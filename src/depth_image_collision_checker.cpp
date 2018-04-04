#include <pips/collision_testing/depth_image_collision_checker.h>



namespace pips
{
  namespace collision_testing
  {
    

    DepthImageCollisionChecker::DepthImageCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh) : 
      PipsCollisionChecker(nh,pnh)
    {
      
    }

    std::shared_ptr<pips::utils::AbstractCameraModel> DepthImageCollisionChecker::getCameraModel()
    {
      cam_model_ = std::make_shared<pips::utils::DepthCameraModel>();
      return cam_model_;
    }
    
    void DepthImageCollisionChecker::setImage(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
      cam_model_->setInfo(info_msg);
      
      PipsCollisionChecker::setImage(image_msg);
    }
    
  
  }
  
}
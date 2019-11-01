#include <pips/collision_testing/depth_image_collision_checker.h>



namespace pips
{
  namespace collision_testing
  {
    

    DepthImageCollisionChecker::DepthImageCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tfm) : 
      PipsCollisionChecker(nh,pnh,name,tfm)
    {
      
    }
    
    DepthImageCollisionChecker::DepthImageCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name) :
      PipsCollisionChecker(nh,pnh,name,tfm)
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
      cam_model_->update(); //Not sure if there was a reason for requiring this additional step
      
      PipsCollisionChecker::setImage(image_msg);
    }
    
  
  }
  
}

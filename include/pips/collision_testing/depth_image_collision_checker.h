#ifndef DEPTH_IMAGE_COLLISION_CHECKER_H
#define DEPTH_IMAGE_COLLISION_CHECKER_H

#include <pips/utils/depth_camera_model.h>
#include <pips/collision_testing/pips_collision_checker.h>

namespace pips
{
  namespace collision_testing
  {
      
    class DepthImageCollisionChecker : public PipsCollisionChecker
    {
    private:
      std::shared_ptr<pips::utils::DepthCameraModel> cam_model_;
      
      
    public:
      
      DepthImageCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, const tf2_utils::TransformManager& tfm=tf2_utils::TransformManager(false));

      
      void setImage(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
      static constexpr const char* DEFAULT_NAME="depth_image_collision_checker";

      
    private:
      std::shared_ptr<pips::utils::AbstractCameraModel> getCameraModel();
      
      
    };
  }
}



#endif //DEPTH_IMAGE_COLLISION_CHECKER_H

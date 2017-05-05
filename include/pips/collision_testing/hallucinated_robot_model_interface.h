#ifndef HALLUCINATED_ROBOT_MODEL_INTERFACE_H
#define HALLUCINATED_ROBOT_MODEL_INTERFACE_H

#include <pips/collision_testing/robot_models/hallucinated_robot_model.h>
#include <dynamic_reconfigure/server.h>
#include <pips/HallucinatedRobotModelConfig.h>

class HallucinatedRobotModelInterface
{
public:
  
  HallucinatedRobotModelInterface(ros::NodeHandle nh, ros::NodeHandle pnh);
  
  void configCB(pips::HallucinatedRobotModelConfig &config, uint32_t level);
  void init();
  
  template <typename T>
  bool testCollision(const T pose)
  {
    boost::mutex::scoped_lock lock(model_mutex_);
    return model_->testCollision(pose);
  }
  
  template <typename T>
  cv::Mat generateHallucinatedRobot(const T pose)
  {
    boost::mutex::scoped_lock lock(model_mutex_);
    return model_->generateHallucinatedRobot(pose);
  }
  
  void updateModel(const cv_bridge::CvImage::ConstPtr& cv_image_ref, const sensor_msgs::CameraInfoConstPtr& info_msg, double scale);
  void setTransform(const geometry_msgs::TransformStamped& base_optical_transform);
  
private:
  
  std::shared_ptr<image_geometry::PinholeCameraModel> cam_model_; // Note: I could get rid of the pointer and pass a reference to the implementation constructor. That would require making sure that cam_model_ was not destructed before model_. I think I've already arranged things properly for that to work, but not worth worrying about
  std::shared_ptr<HallucinatedRobotModelBase> model_;

  double scale_;
  bool show_im_=false;
  
  int model_type_ = -1;
  
  boost::mutex model_mutex_;  //This class manages all calls to the implementation, so the mutex is also stored here
  std::string name_ = "HallucinatedRobotModelInterface";
  
  cv_bridge::CvImage::ConstPtr cv_image_ref_;
  
  ros::NodeHandle nh_, pnh_;
  geometry_msgs::TransformStamped base_optical_transform_;
  
  typedef dynamic_reconfigure::Server<pips::HallucinatedRobotModelConfig> ReconfigureServer;
  std::shared_ptr<ReconfigureServer> reconfigure_server_;
  
  
};

#endif /* HALLUCINATED_ROBOT_MODEL_INTERFACE_H */

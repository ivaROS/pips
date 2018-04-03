#ifndef HALLUCINATED_ROBOT_MODEL_INTERFACE_H
#define HALLUCINATED_ROBOT_MODEL_INTERFACE_H

#include <pips/collision_testing/robot_models/hallucinated_robot_model.h>
#include <dynamic_reconfigure/server.h>
#include <pips/HallucinatedRobotModelConfig.h>

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>

typedef boost::shared_mutex Mutex;
typedef boost::unique_lock< Mutex > WriteLock;
typedef boost::shared_lock< Mutex > ReadLock;

class HallucinatedRobotModelInterface
{
public:
  
  HallucinatedRobotModelInterface(ros::NodeHandle nh, ros::NodeHandle pnh);
  
  void configCB(pips::HallucinatedRobotModelConfig &config, uint32_t level);
  void init();
  
  void setCameraModel(std::shared_ptr<pips::utils::AbstractCameraModel> cam_model);
  
  template <typename T>
  CCResult testCollision(const T pose, CCOptions options)
  {
    ReadLock lock(model_mutex_);
    return model_->testCollision(pose, options);
  }
  
  template <typename T>
  cv::Mat generateHallucinatedRobot(const T pose)
  {
    ReadLock lock(model_mutex_);
    return model_->generateHallucinatedRobot(pose);
  }
  
  void updateModel(const cv_bridge::CvImage::ConstPtr& cv_image_ref, double scale);
  void setTransform(const geometry_msgs::TransformStamped& base_optical_transform);
  
  
private:
  
  std::shared_ptr<pips::utils::AbstractCameraModel> cam_model_; // Note: I could get rid of the pointer and pass a reference to the implementation constructor. That would require making sure that cam_model_ was not destructed before model_. I think I've already arranged things properly for that to work, but not worth worrying about
  std::shared_ptr<HallucinatedRobotModelBase> model_;

  double scale_;
  bool show_im_=false;
  
  int model_type_ = -1;
  
  Mutex model_mutex_; // Allows simultaneous read operations; prevents anything else from happening while writing 
  std::string name_ = "HallucinatedRobotModelInterface";
  
  cv_bridge::CvImage::ConstPtr cv_image_ref_;
  
  ros::NodeHandle nh_, pnh_;
  geometry_msgs::TransformStamped base_optical_transform_;
  
  typedef dynamic_reconfigure::Server<pips::HallucinatedRobotModelConfig> ReconfigureServer;
  std::shared_ptr<ReconfigureServer> reconfigure_server_;
  
  
};

#endif /* HALLUCINATED_ROBOT_MODEL_INTERFACE_H */

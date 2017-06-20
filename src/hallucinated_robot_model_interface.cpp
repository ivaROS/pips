#include "pips/collision_testing/hallucinated_robot_model_interface.h"
#include <pips/collision_testing/robot_models/rectangular_model.h>
#include <pips/collision_testing/robot_models/rectangular_model_ss.h>
#include <pips/collision_testing/robot_models/rectangular_model_ocl.h>
#include <pips/collision_testing/robot_models/cylindrical_model.h>
#include <pips/collision_testing/robot_models/cylindrical_model_t.h>

#include <pips/HallucinatedRobotModelConfig.h>

  
  HallucinatedRobotModelInterface::HallucinatedRobotModelInterface(ros::NodeHandle nh, ros::NodeHandle pnh) :
    nh_(nh), pnh_(pnh, name_)
  {
    cam_model_ = std::make_shared<image_geometry::PinholeCameraModel>();
    reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
    //pnh_ = ros::NodeHandle(nh_, name_);
    
  }
  
  void HallucinatedRobotModelInterface::init()
  {
    reconfigure_server_->setCallback(boost::bind(&HallucinatedRobotModelInterface::configCB, this, _1, _2));
  }
  
  void HallucinatedRobotModelInterface::configCB(pips::HallucinatedRobotModelConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM_NAMED(name_, "Reconfigure Request: "); // TODO: print out the model type and other parameter values
    
    boost::mutex::scoped_lock lock(model_mutex_); /* Mutex prevents dynamic reconfigure from changing anything while model in use */
    
    /* if the model type in the reconfigure request is different than the previous, we need to instantiate the new one */
    if(config.model_type != model_type_)
    {
      if(config.model_type == pips::HallucinatedRobotModel_rectangular)
      {
        model_ = std::make_shared<RectangularModel>();
      }
      else if(config.model_type == pips::HallucinatedRobotModel_rectangular_ocl)
      {
        model_ = std::make_shared<RectangularModelOCL>();
      }
      else if(config.model_type == pips::HallucinatedRobotModel_rectangular_ss)
      {
        model_ = std::make_shared<RectangularModelSS>();
      }
      else if (config.model_type == pips::HallucinatedRobotModel_cylindrical)
      {
        model_ = std::make_shared<CylindricalModel>();
      }
      else if (config.model_type == pips::HallucinatedRobotModel_cylindrical_t)
      {
        model_ = std::make_shared<CylindricalModelT>();
      }
      else if (config.model_type == pips::HallucinatedRobotModel_cylindrical_c)
      {
        model_ = std::make_shared<HallucinatedRobotModelCacheWrapper<CylindricalModel> >();
      }
      
      /*
      else if (config.model_type == pips::HallucinatedRobotModel_rectangular_min)
      {
        model_ = std::make_shared<RectangularModelMinV>();
      }
      */
/*      else if (config.model_type == pips::HallucinatedRobotModel_dense)
      {
        ROS_WARN_NAMED(name_, "Sorry, this model is currently disabled. No changes have been made.");
        return;
        //model_ = std::make_shared<DenseModel>(nh_, pnh_);
      }
*/      
      model_->init(cam_model_, base_optical_transform_);
      
      ROS_WARN_STREAM_NAMED(name_, "transform: " << toString(base_optical_transform_));
      
      if(cv_image_ref_)
      {
        model_->updateModel(cv_image_ref_, scale_);  // catches initial condition where image_ref_ is empty
      }
      else
      {
        ROS_WARN_NAMED(name_, "cv_image_ref_ was NULL");
      }
      
      model_type_ = config.model_type;
    }
    
    model_->setParameters(config.robot_radius, config.robot_height, config.floor_tolerance, config.safety_expansion, config.show_im);
    

  }
  
  
  void HallucinatedRobotModelInterface::updateModel(const cv_bridge::CvImage::ConstPtr& cv_image_ref, const sensor_msgs::CameraInfoConstPtr& info_msg, double scale)
  {
    cv_image_ref_ = cv_image_ref;
    scale_ = scale;
    
    {
      boost::mutex::scoped_lock lock(model_mutex_);
      model_->updateModel(cv_image_ref_, scale);
      cam_model_->fromCameraInfo(info_msg); //We are only updating the contents of cam_model_, rather than the object it points to, so no need to pass it to the model again

    }
  }
  
  void HallucinatedRobotModelInterface::setTransform(const geometry_msgs::TransformStamped& base_optical_transform)
  {
    base_optical_transform_ = base_optical_transform;
  }
  


#ifndef HALLUCINATED_ROBOT_MODEL_H
#define HALLUCINATED_ROBOT_MODEL_H


#include <pips/HallucinatedRobotModelConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
//#include <opencv/cv.h>
#include <Eigen/Eigen>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


class HallucinatedRobotModelImpl
{

    public: 
    
    HallucinatedRobotModelImpl()
      {
        name_ = getName();
      }
    
    virtual bool testCollision(const cv::Point3d pt)=0;
    virtual cv::Mat generateHallucinatedRobot(const cv::Point3d pt)
    {
      return image_ref_.clone();
    }
    
    virtual void setParameters(double robot_radius, double robot_height, double floor_tolerance, double safety_expansion, bool show_im)=0;

    virtual void init(std::shared_ptr<image_geometry::PinholeCameraModel> cam_model)
    {
      cam_model_ = cam_model;
    }
    
    virtual void updateModel(cv::Mat& image, double scale)
    {
      image_ref_ = image;
      scale_ = scale;
      
      if(show_im_)
      {
        double min;
        double max;
        cv::minMaxIdx(image_ref_, &min, &max);
        cv::Mat adjIm;
        cv::convertScaleAbs(image_ref_, adjIm, 255 / max);
        
        cv::imshow("Original image", adjIm);
        cv::waitKey(1);
      }
    }
    
    virtual std::string getName(){ return "undefined"; }

    
    protected:

    std::shared_ptr<image_geometry::PinholeCameraModel> cam_model_;
    cv::Mat image_ref_;
    double robot_radius_, robot_height_, floor_tolerance_;
    double scale_;
    bool show_im_=false;
    std::string name_;


};


class RectangularModel : public HallucinatedRobotModelImpl
{
    private:
    std::vector<cv::Point3d> co_offsets_;
    
    public:
    bool testCollision(const cv::Point3d pt);
    cv::Mat generateHallucinatedRobot(const cv::Point3d pt);
    std::string getName() { return "RectangularModel"; }
    virtual void setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im);
    
    private:
    bool isLessThan(const cv::Mat& image, float depth);
};


class CylindricalModel : public HallucinatedRobotModelImpl
{
    private:
    cv::Rect getColumn(const cv::Mat& image, const cv::Point2d& top, const cv::Point2d& bottom);
    
    public:
    bool testCollision(const cv::Point3d pt);
    cv::Mat generateHallucinatedRobot(const cv::Point3d pt);
    std::string getName() { return "CylindricalModel"; }
    virtual void setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im);
    
    virtual void updateModel(cv::Mat& image, double scale);
    
};


class HallucinatedRobotModel
{
  public:
  
  HallucinatedRobotModel(ros::NodeHandle nh)
  {
    cam_model_ = std::make_shared<image_geometry::PinholeCameraModel>();
    nh_ = ros::NodeHandle(nh, name_);
    
    reconfigure_server_ = std::make_shared<ReconfigureServer>(nh_);
    reconfigure_server_->setCallback(boost::bind(&HallucinatedRobotModel::configCB, this, _1, _2));
    
  }
  
  void configCB(pips::HallucinatedRobotModelConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM_NAMED(name_, "Reconfigure Request:");
    
    boost::mutex::scoped_lock lock(model_mutex_); /* Mutex prevents dynamic reconfigure from changing anything while model in use */
    
    /* if the model type in the reconfigure request is different than the previous, we need to instantiate the new one */
    if(config.model_type != model_type_)
    {
      if(config.model_type == pips::HallucinatedRobotModel_rectangular)
      {
        model_ = std::make_shared<RectangularModel>();
      }
      else if (config.model_type == pips::HallucinatedRobotModel_cylindrical)
      {
        model_ = std::make_shared<CylindricalModel>();
      }
      
      model_->init(cam_model_);
      
      if(!image_ref_.empty())
      {
        model_->updateModel(image_ref_, scale_);  // catches initial condition where image_ref_ is empty
      }
      
      model_type_ = config.model_type;
    }
    
    model_->setParameters(config.robot_radius, config.robot_height, config.floor_tolerance, config.safety_expansion, config.show_im);
    

  }
  
  bool testCollision(const cv::Point3d pt)
  {
    boost::mutex::scoped_lock lock(model_mutex_);
    return model_->testCollision(pt);
  }
  
  cv::Mat generateHallucinatedRobot(const cv::Point3d pt)
  {
    boost::mutex::scoped_lock lock(model_mutex_);
    return model_->generateHallucinatedRobot(pt);
  }
  
  void updateModel(cv::Mat& image, const sensor_msgs::CameraInfoConstPtr& info_msg, double scale)
  {
    scale_ = scale;
    cam_model_->fromCameraInfo(info_msg);
    image_ref_ = image;
    
    {
      boost::mutex::scoped_lock lock(model_mutex_);
      model_->updateModel(image, scale);
    }
  }
  
  private:
  
  std::shared_ptr<image_geometry::PinholeCameraModel> cam_model_; // Note: I could get rid of the pointer and pass a reference to the implementation constructor. That would require making sure that cam_model_ was not destructed before model_. I think I've already arranged things properly for that to work, but not worth worrying about
  std::shared_ptr<HallucinatedRobotModelImpl> model_;
  cv::Mat image_ref_;
  double scale_;
  bool show_im_=false;
  
  int model_type_ = -1;
  
  boost::mutex model_mutex_;
  std::string name_ = "HallucinatedRobotModel";
  
  ros::NodeHandle nh_;
  
  typedef dynamic_reconfigure::Server<pips::HallucinatedRobotModelConfig> ReconfigureServer;
  std::shared_ptr<ReconfigureServer> reconfigure_server_;
  
};

#endif /*HALLUCINATED_ROBOT_MODEL_H*/

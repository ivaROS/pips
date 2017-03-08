#ifndef RECTANGULAR_MODEL_H
#define RECTANGULAR_MODEL_H


#include <pips/HallucinatedRobotModelConfig.h>
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
    
    if(config.model_type != model_type_)
    {
      if(config.model_type == pips::HallucinatedRobotModelConfig::model_enum::rectangular)
      {
        model_ = std::make_shared<RectangularModel>(config.radius, config.robot_height, config.floor_tolerance, config.safety_expansion)
      }
      else if (config.model_type == pips::HallucinatedRobotModelConfig::model_enum::cylindrical)
      {
        model_ = std::make_shared<CylindricalModel>(config.radius, config.robot_height, config.floor_tolerance, config.safety_expansion)
      }
    }
    else
    {
      model_->updateParameters(config.robot_radius, config.robot_height, config.floor_tolerance, config.safety_expansion, config.show_im);
    }
  }
  
  bool testCollision(const cv::Point3d pt)
  {
    return model_->testCollision(pt);
  }
  
  cv::Mat generateHallucinatedRobot(const cv::Point3d pt)
  {
    return model_->generateHallucinatedRobot(pt);
  }
  
  void updateModel(cv::Mat& image, const sensor_msgs::CameraInfoConstPtr& info_msg, double scale)
  {
    scale_ = scale;
    cam_model_->fromCameraInfo(info_msg);
    image_ref_ = image;
    
    model_->updateModel(image, scale);
  }
  
  private:
  std::shared_ptr<HallucinatedRobotModelImpl> model_;
  std::shared_ptr<image_geometry::PinholeCameraModel> cam_model_;
  cv::Mat image_ref_;
  double scale_;
  bool show_im_=false;
  
  int model_type_ = -1;
  
  std::string name_ = "HallucinatedRobotModel";
  
  ros::NodeHandle nh_;
  
  typedef dynamic_reconfigure::Server<pips::HallucinatedRobotModelConfig> ReconfigureServer;
  std::shared_ptr<ReconfigureServer> reconfigure_server_;
  
}

class HallucinatedRobotModelImpl
{

    public: 
    virtual bool testCollision(const cv::Point3d pt)=0;
    virtual cv::Mat generateHallucinatedRobot(const cv::Point3d pt)
    {
      return image_ref_.clone();
    }
    
    void updateParameters(double robot_radius, double robot_height, double floor_tolerance, double safety_expansion, bool show_im)
    {
      robot_radius_ = robot_radius;
      robot_height_ = robot_height;
      floor_tolerance_ = floor_tolerance;
      safety_expansion_ = safety_expansion;
      show_im_ = show_im;
    }
    
    void updateModel(cv::Mat& image, double scale)
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

    
    protected:

    std::shared_ptr<image_geometry::PinholeCameraModel> cam_model_;
    cv::Mat image_ref_;
    double robot_radius_, robot_height_, floor_tolerance_;
    double scale_;
    bool show_im_=false;
    boost::mutex parameter_mutex_;



};


class RectangularModel : public HallucinatedRobotModelImpl
{
    std::vector<cv::Point3d> co_offsets_;
    public:
    RectangularModel::RectangularModel(){};
    RectangularModel(double radius, double height, double safety_expansion, double floor_tolerance);
    bool testCollision(const cv::Point3d pt);
    cv::Mat generateHallucinatedRobot(const cv::Point3d pt);
};


class CylindricalModel : public HallucinatedRobotModelImpl
{
    private:
    cv::Rect getColumn(const cv::Mat& image, const cv::Point2d& top, const cv::Point2d& bottom);
    public:
    CylindricalModel(double radius, double height, double safety_expansion, double floor_tolerance);
    bool testCollision(const cv::Point3d pt);
    cv::Mat generateHallucinatedRobot(const cv::Point3d pt);

};

#endif /*RECTANGULAR_MODEL_H*/

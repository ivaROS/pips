#ifndef HALLUCINATED_ROBOT_MODEL_H
#define HALLUCINATED_ROBOT_MODEL_H


#include <pips/HallucinatedRobotModelConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
//#include <opencv/cv.h>
#include <Eigen/Eigen>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_eigen/tf2_eigen.h>

#include <tf/transform_datatypes.h> //For creating quaternion easily

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


// Only needed for dense model. Really, each class should have its own header
#include <tf2_ros/transform_listener.h>
#include <mesh_filter/depth_model.h>


typedef geometry_msgs::Pose PoseType;

// should really put these in some sort of namespace... and probably into a separate header

  template<typename T>
  void convertPose(const T& A, T& B)
  {
    B = A;
  }
  
  inline
  void convertPose(const PoseType pose_in, cv::Point3d& pose_out)
  {
      pose_out = cv::Point3d(pose_in.position.x, pose_in.position.y, pose_in.position.z);
  }
  
  inline
  void convertPose(double pose_in[], PoseType& pose_out)
  {
    pose_out.position.x = pose_in[0];
    pose_out.position.y = pose_in[1];
    pose_out.position.z = pose_in[2];
    
    if(sizeof(pose_in) / sizeof(double) == 4)
    {
      pose_out.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose_in[3]);
    }
  }


  inline
  void convertPose(const Eigen::Affine3d& in, geometry_msgs::Pose& msg) {
    msg = tf2::toMsg(in);
  //tf2::fromMsg(*currentPose_, pose); // This should work with most recent version of tf2_eigen
   /*  msg.position.x = in.translation().x();
     msg.position.y = in.translation().y();
     msg.position.z = in.translation().z();
     msg.orientation.x = Eigen::Quaterniond(in.rotation()).x();
     msg.orientation.y = Eigen::Quaterniond(in.rotation()).y();
     msg.orientation.z = Eigen::Quaterniond(in.rotation()).z();
     msg.orientation.w = Eigen::Quaterniond(in.rotation()).w(); */
  }
  
  /* Note: not sure whether this one actually works */
  inline
  void convertPose(const geometry_msgs::Pose& msg, Eigen::Affine3d& out)
  {
    tf2::fromMsg(msg, out);
  }
  
  inline
  void convertPose(const geometry_msgs::Pose& msg, geometry_msgs::Pose& out)
  {
    out = msg;
  }


class HallucinatedRobotModelBase
{
  public:
    
    HallucinatedRobotModelBase()
    {
      name_ = getName();
    }
    

    virtual bool testCollision(geometry_msgs::Pose pose)=0;
    virtual cv::Mat generateHallucinatedRobot(const geometry_msgs::Pose pose)=0;
    virtual void setParameters(double robot_radius, double robot_height, double floor_tolerance, double safety_expansion, bool show_im)=0;
    
    void updateModel(cv_bridge::CvImage::ConstPtr& cv_image_ref, double scale)
    {
      cv_image_ref_ = cv_image_ref;
      image_ref_ = getImage(cv_image_ref);
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
    
    
    void init(std::shared_ptr<image_geometry::PinholeCameraModel>& cam_model)
    {
      cam_model_ = cam_model;
    }
    

  
  protected:
    virtual std::string getName(){ return "undefined"; }
    
    virtual cv::Mat getImage(cv_bridge::CvImage::ConstPtr& cv_image_ref)
    {
      return cv_image_ref->image;
    }
   
  protected:
    std::shared_ptr<image_geometry::PinholeCameraModel> cam_model_;
    cv::Mat image_ref_; //Allows method to make local changes if needed
    cv_bridge::CvImage::ConstPtr cv_image_ref_; //Allows access to original data and msg info
    double robot_radius_, robot_height_, floor_tolerance_;
    double scale_;
    bool show_im_=false;
    std::string name_;
    

};

template<typename S> class HallucinatedRobotModelImpl : public HallucinatedRobotModelBase
{

    public: 
    
    HallucinatedRobotModelImpl() : HallucinatedRobotModelBase()
    {

    }
    
    bool testCollision(const geometry_msgs::Pose pose)
    {
      S convertedPose;
      convertPose(pose, convertedPose);
      return testCollisionImpl(convertedPose);
    }
         
    cv::Mat generateHallucinatedRobot(const geometry_msgs::Pose pose)
    {
      S convertedPose;
      convertPose(pose, convertedPose);
      return generateHallucinatedRobotImpl(convertedPose);
    }


    
protected:
    virtual bool testCollisionImpl(const S pose)=0;
    
    virtual cv::Mat generateHallucinatedRobotImpl(const S pose)
    {
      return image_ref_.clone();
    }

};


class RectangularModel : public HallucinatedRobotModelImpl<cv::Point3d>
{
  private:
    std::vector<cv::Point3d> co_offsets_;
    
  public:
    void setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im);


  protected:
/*
    virtual bool testCollision(const PoseType pose)
    {
      return testCollision(cv::Point3d(pose.position.x, pose.position.y, pose.position.z));
    }
  */  
    bool testCollisionImpl(const cv::Point3d pt);
    
    /*
    virtual cv::Mat generateHallucinatedRobot(const PoseType pose)
    {
      return generateHallucinatedRobot(cv::Point3d(pose.position.x, pose.position.y, pose.position.z));
    }
    */
    
    cv::Mat generateHallucinatedRobotImpl(const cv::Point3d pt);
    
    std::string getName() { return "RectangularModel"; }

    
  private:
    bool isLessThan(const cv::Mat& image, float depth);
};


class CylindricalModel : public HallucinatedRobotModelImpl<cv::Point3d>
{
  private:
    cv::Rect getColumn(const cv::Mat& image, const cv::Point2d& top, const cv::Point2d& bottom);
    
  public:
    void setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im);
    /*
    virtual bool testCollision(const PoseType pose)
    {
      return testCollision(cv::Point3d(pose.position.x, pose.position.y, pose.position.z));
    }
    */
   
  protected:
  
    bool testCollisionImpl(const cv::Point3d pt);
    
    /*
    virtual cv::Mat generateHallucinatedRobot(const PoseType pose)
    {
      return generateHallucinatedRobot(cv::Point3d(pose.position.x, pose.position.y, pose.position.z));
    }
    */
    
    cv::Mat generateHallucinatedRobotImpl(const cv::Point3d pt);
    
    std::string getName() { return "CylindricalModel"; }

    cv::Mat getImage(cv_bridge::CvImage::ConstPtr& cv_image_ref);

};

class DenseModel : public HallucinatedRobotModelImpl<geometry_msgs::Pose>
{
  public:
    DenseModel(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~DenseModel();
    

    void setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im);
    
  protected:
    bool testCollisionImpl(const geometry_msgs::Pose pose);
    cv::Mat generateHallucinatedRobotImpl(const geometry_msgs::Pose pose);
    std::string getName() { return "DenseModel"; }
    
  private:
    bool isReady();
    
  private:
    std::shared_ptr<depth_projection::DepthModel> depth_model_;
    
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    ros::NodeHandle nh_, pnh_;
    
};



class HallucinatedRobotModelInterface
{
public:
  
  HallucinatedRobotModelInterface(ros::NodeHandle nh, ros::NodeHandle pnh);
  
  void configCB(pips::HallucinatedRobotModelConfig &config, uint32_t level);
  
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
  
  typedef dynamic_reconfigure::Server<pips::HallucinatedRobotModelConfig> ReconfigureServer;
  std::shared_ptr<ReconfigureServer> reconfigure_server_;
  
  
};

#endif /*HALLUCINATED_ROBOT_MODEL_H*/

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
//#include <tf2_ros/transform_listener.h>
//#include <mesh_filter/depth_model.h>


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
      
      doPrecomputation();
      
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
    
    
    void init(std::shared_ptr<image_geometry::PinholeCameraModel>& cam_model, const geometry_msgs::TransformStamped& base_optical_transform)
    {
      cam_model_ = cam_model;
      base_optical_transform_ = base_optical_transform;
    }
    

  
  protected:
    virtual std::string getName(){ return "undefined"; }
    
    virtual cv::Mat getImage(cv_bridge::CvImage::ConstPtr& cv_image_ref)
    {
      return cv_image_ref->image;
    }
    
    virtual void doPrecomputation() {}
   
  protected:
    std::shared_ptr<image_geometry::PinholeCameraModel> cam_model_;
    cv::Mat image_ref_; //Allows method to make local changes if needed
    cv_bridge::CvImage::ConstPtr cv_image_ref_; //Allows access to original data and msg info
    double robot_radius_, robot_height_, floor_tolerance_;
    double scale_;
    bool show_im_=false;
    std::string name_;
    geometry_msgs::TransformStamped base_optical_transform_;
    

};

template<typename S> class HallucinatedRobotModelImpl : public HallucinatedRobotModelBase
{

    public: 
    
    HallucinatedRobotModelImpl() : HallucinatedRobotModelBase()
    {

    }
    
    bool testCollision(const geometry_msgs::Pose pose)
    {
      geometry_msgs::Pose pose_t = transformPose(pose);
      S convertedPose;
      convertPose(pose_t, convertedPose);
      return testCollisionImpl(convertedPose);
    }
         
    cv::Mat generateHallucinatedRobot(const geometry_msgs::Pose pose)
    {
      geometry_msgs::Pose pose_t = transformPose(pose);
      S convertedPose;
      convertPose(pose_t, convertedPose);
      return generateHallucinatedRobotImpl(convertedPose);
    }
    



    
protected:
    virtual bool testCollisionImpl(const S pose)=0;
    
    virtual cv::Mat generateHallucinatedRobotImpl(const S pose)
    {
      return image_ref_.clone();
    }
    
    virtual geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose)
    {
    // It would probably be better not to convert from whatever to geometry_msgs::Pose to Eigen::Affine3d back to geometry and then to whatever... But this seems the cleanest
      Eigen::Affine3d pose_eig;
      tf2::fromMsg(pose, pose_eig);

      //Transform coordinates of robot base into camera's optical frame
      Eigen::Affine3d pose_eig_t;
      
      tf2::doTransform(pose_eig, pose_eig_t, base_optical_transform_);
      
      geometry_msgs::Pose pose_t;
      convertPose(pose_eig_t, pose_t);
      
      ROS_DEBUG_STREAM_NAMED(name_, "Pose [" << 
        pose.position.x << "," << pose.position.y << "," << pose.position.z << "] (" <<
        pose.orientation.w << "," << pose.orientation.x << "," << pose.orientation.y << "," << pose.orientation.z << 
        ") transformed to [" << 
        pose_t.position.x << "," << pose_t.position.y << "," << pose_t.position.z << "] (" << 
        pose_t.orientation.w << "," << pose_t.orientation.x << "," << pose_t.orientation.y << "," << pose_t.orientation.z 
        << ")"
        );
      
      return pose_t;
      
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
    virtual bool testCollisionImpl(const cv::Point3d pt);
    
    /*
    virtual cv::Mat generateHallucinatedRobot(const PoseType pose)
    {
      return generateHallucinatedRobot(cv::Point3d(pose.position.x, pose.position.y, pose.position.z));
    }
    */
    
    
    virtual cv::Mat generateHallucinatedRobotImpl(const cv::Point3d pt);
    
    virtual std::string getName() { return "RectangularModel"; }

    
  private:
    bool isLessThan(const cv::Mat& image, float depth);
};

class RectangularModelMinV : public RectangularModel
{
  protected:  
    double min_dist_ = -1;
    
  protected:
    void doPrecomputation()
    {
      cv::minMaxLoc(image_ref_, &min_dist_);  // This isn't enough: the floor is detected. Has to be the minimum value above plane.
      ROS_INFO_STREAM_NAMED(name_, "New minimum = " << min_dist_);
    }
  
    bool testCollisionImpl(const cv::Point3d pt)
    {
      if(pt.x + robot_radius_ < min_dist_ * scale_)
      {
        ROS_DEBUG_NAMED(name_, "Point in known free space");
        return false;
      }
      else
      {
        return RectangularModel::testCollisionImpl(pt);
      }

    }
    
    std::string getName() { return "RectangularModelMinV"; }
    
};



/*
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
    geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose);
    
  private:
    bool isReady();
    
  private:
    std::shared_ptr<depth_projection::DepthModel> depth_model_;
    
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    ros::NodeHandle nh_, pnh_;
    std::shared_ptr<image_transport::ImageTransport> model_depth_transport_;
    image_transport::CameraPublisher pub_model_depth_image_;
    
    bool isLessThan(const cv::Mat& depth_im, const cv::Mat& generated_im);
};
*/


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

#endif /*HALLUCINATED_ROBOT_MODEL_H*/

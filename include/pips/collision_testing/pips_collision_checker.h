#ifndef PIPS_COLLISION_CHECKER_H
#define PIPS_COLLISION_CHECKER_H

#include <pips/collision_testing/transforming_collision_checker.h>
#include <pips/utils/depth_camera_model.h>

#include "pips/collision_testing/hallucinated_robot_model_interface.h"
#include <pips/GenerateDepthImage.h>

#include <sensor_msgs/Image.h>
//#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry> // <Eigen/Eigen>
#include <image_geometry/pinhole_camera_model.h>
#include <chrono>
#include <memory>

//Things that could be forward declared:
#include <image_geometry/pinhole_camera_model.h>


//class cv::Mat;




class PipsCollisionChecker : public pips::collision_testing::TransformingCollisionChecker
{



public :
    //Needed to ensure that eigen objects aligned
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PipsCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME);

    void setImage(const sensor_msgs::ImageConstPtr& image_msg);
    CCResult testCollisionImpl(PoseType pose, CCOptions options=CCOptions());
    cv::Mat generateDepthImage(PoseType pose);
    
    void initImpl();
    
    
    void setTransform(const geometry_msgs::TransformStamped& base_optical_transform);
    
    static constexpr const char* DEFAULT_NAME="pips_collision_checker";

    
    //void generateImageCoord(const double xyz[], double * uv);
    /*
    template<typename T>
    bool testCollision(T pose_in)
    {
      PoseType pose_out;
      convertPose(pose_in, pose_out);
      return testCollision(pose_out);
    }

    template<typename T>
    cv::Mat generateDepthImage(T pose_in)
    {
      PoseType pose_out;
      convertPose(pose_in, pose_out);
      return generateDepthImage(pose_out);
    }
    */
    
protected:
  ros::Publisher posepub_, pointpub_;
  

private:
    bool getDepthImageSrv(pips::GenerateDepthImage::Request &req, pips::GenerateDepthImage::Response &res);
    virtual std::shared_ptr<pips::utils::AbstractCameraModel> getCameraModel()=0;
    
    
private :
    std::string name_ = "PipsCollisionChecker";
    
    //ros::ServiceClient checker_; 


    //image_transport::Publisher depthpub_;
    std::shared_ptr<pips::utils::AbstractCameraModel> cam_model_;
    ros::NodeHandle nh_, pnh_;

    //image_transport::ImageTransport it_; // Needs to be after node handles to ensure they are initialized first

    
    Eigen::Affine3d optical_transform_;
    geometry_msgs::TransformStamped base_optical_transform_;
    
    ros::ServiceServer depth_generation_service_;
    
    HallucinatedRobotModelInterface robot_model_;
    
    unsigned int scale_;
    
    cv::Mat image_,image_ref_;
    
    cv_bridge::CvImageConstPtr input_bridge_ref_;
    
    cv_bridge::CvImagePtr input_bridge_, output_bridge_;
    pips::utils::DurationAccumulator setup_durations_;


    
} ;
/*
    template<typename T, typename S>
    void getPose(T pose_in, S pose_out)
    {
      tf2::fromMsg(pose_in, pose_out);
      return pose_out;
    }
    
    template<typename T>
    PoseType getPose(T pose_in)
    {
      PoseTyp pose_out;
      getPose(pose_in, pose_out);
      return pose_out;
    }
*/


    //PoseType getPose(geometry_msgs::Point point);        
    //eigen::Affine3d getPose(geometry_msgs::Pose pose);

#endif /* PIPS_COLLISION_CHECKER_H */


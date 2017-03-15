#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H


#include "hallucinated_robot_model.h"
#include <pips/GenerateDepthImage.h>

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv/cv.h>
#include <Eigen/Eigen>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <memory>

/* This file currently is designed to work as though it were a generic interface but is specific to pips. 
  The code will be separated into interface and implementation specific versions later   
*/
// Ex: this struct definition must be in the generic interface, which will be included by derived classes such as pips
struct SensorData
{
  virtual const std_msgs::Header getHeader() = 0;
};

typedef std::shared_ptr<SensorData> SensorDataPtr;

struct DepthData : public SensorData
{
  public:
  const sensor_msgs::Image::ConstPtr image_msg;
  const sensor_msgs::CameraInfo::ConstPtr info_msg;
  
  DepthData(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& info_msg) :
    image_msg(image_msg), info_msg(info_msg)
  {}
  
  virtual const std_msgs::Header getHeader()
  {
    return info_msg->header;
  }
  
};

typedef std::shared_ptr<DepthData> DepthDataPtr;

class CollisionChecker
{



public :
    CollisionChecker(geometry_msgs::TransformStamped& optical_transform, std::vector<cv::Point3d> co_points, bool pub_image);
    CollisionChecker(geometry_msgs::TransformStamped& optical_transform);

    void setImage(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
    void setSensorData(const SensorDataPtr& depth_data);
    bool testCollision(double xyz[] );
    cv::Mat generateDepthImage(double xyz[] );

    //
    void generateImageCoord(const double xyz[], double * uv);

private :
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher depthpub_;
    ros::Publisher posepub_;
    std::shared_ptr<image_geometry::PinholeCameraModel> cam_model_;

    Eigen::Affine3d optical_transform_;
    
    ros::ServiceServer depth_generation_service_;
    
    HallucinatedRobotModel robot_model_;
    
    bool publish_image_;
    unsigned int scale_;
    
    cv::Mat image_,image_ref_;
    cv_bridge::CvImagePtr input_bridge_, output_bridge_;
    std::chrono::duration<double, std::milli> total_duration;
    
    bool getDepthImage(pips::GenerateDepthImage::Request &req, pips::GenerateDepthImage::Response &res);

} ;

#endif /* COLLISION_CHECKER_H */


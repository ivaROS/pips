#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv/cv.h>
#include <Eigen/Eigen>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

class CollisionChecker
{



   public :
        CollisionChecker(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg, geometry_msgs::TransformStamped& optical_transform, std::vector<cv::Point3d> co_points, bool gen_image);
        
        bool testCollision(double xyz[] );
        void setBaseTransform(geometry_msgs::TransformStamped& base_transform);
        bool testCollision(const Eigen::Vector3d origin_r);
   private :
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher depthpub_;
  image_geometry::PinholeCameraModel cam_model_;

  Eigen::Affine3d base_to_optical_transform_, optical_transform_;


  std::vector<cv::Point3d> co_offsets_;
  bool generate_image_;
  int scale_;
  
  cv::Mat image_,image_ref_;
  cv_bridge::CvImagePtr input_bridge_, output_bridge_;
  std::chrono::duration<double, std::milli> total_duration;
        
} ;

#endif /* COLLISION_CHECKER_H */


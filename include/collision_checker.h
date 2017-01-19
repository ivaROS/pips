#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H


#include "rectangular_model.h"

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
    CollisionChecker(geometry_msgs::TransformStamped& optical_transform, std::vector<cv::Point3d> co_points, bool pub_image);
    CollisionChecker(geometry_msgs::TransformStamped& optical_transform, std::shared_ptr<HallucinatedRobotModel> model, bool pub_image);

    void setImage(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
    bool testCollision(double xyz[] );
    cv::Mat generateDepthImage(double xyz[] );

    //
    void generateImageCoord(const double xyz[], double * uv);

private :
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher depthpub_;
    ros::Publisher posepub_;
    image_geometry::PinholeCameraModel cam_model_;

    Eigen::Affine3d optical_transform_;
    
    std::shared_ptr<HallucinatedRobotModel> robot_model_;
    
    bool publish_image_;
    unsigned int scale_;
    
    cv::Mat image_,image_ref_;
    cv_bridge::CvImagePtr input_bridge_, output_bridge_;
    std::chrono::duration<double, std::milli> total_duration;

} ;

#endif /* COLLISION_CHECKER_H */


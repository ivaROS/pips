#ifndef PIPS_COLLISION_CHECKER_H
#define PIPS_COLLISION_CHECKER_H

#include <pips/collision_testing/general_collision_checker.h>
//#include <pips/utils/depth_camera_model.h>

#include <pips/utils/abstract_camera_model.h>
#include <pips/GenerateDepthImage.h>
#include <pips/utils/image_comparison_result.h>

#include <sensor_msgs/Image.h>
//#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/TransformStamped.h>
//#include <Eigen/Geometry> // <Eigen/Eigen>
#include <chrono>
#include <memory>





class PipsCollisionChecker : public pips::collision_testing::GeneralCollisionChecker
{



public :
    //Needed to ensure that eigen objects aligned
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PipsCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, const tf2_utils::TransformManager& manager=tf2_utils::TransformManager(false));
    PipsCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& manager, const std::string& name=DEFAULT_NAME);
    
    void setImage(const sensor_msgs::ImageConstPtr& image_msg);
    CCResult testCollisionImpl(PoseType pose, CCOptions options=CCOptions());
    cv::Mat generateDepthImage(PoseType pose);
    
    virtual void initImpl();
    virtual std_msgs::Header getCurrentHeader();
    
    
protected:
    cv::Rect getColumnRect(const int x, const int y, const int width, const int height);
    cv::Rect getColumnRect(const cv::Rect& rect);
    ComparisonResult imageSpaceCollisionImpl(const geometry_msgs::Pose pose, CCOptions options);
    ComparisonResult isLessThan(const cv::Mat& col, float depth);
    ComparisonResult isLessThanDetails(const cv::Mat& col, float depth);
  
    
    static constexpr const char* DEFAULT_NAME="pips_collision_checker";
    
protected:
  ros::Publisher posepub_, pointpub_;
  

private:
    bool getDepthImageSrv(pips::GenerateDepthImage::Request &req, pips::GenerateDepthImage::Response &res);
    virtual std::shared_ptr<pips::utils::AbstractCameraModel> getCameraModel()=0;
    
    
protected :
    
    std::shared_ptr<pips::utils::AbstractCameraModel> cam_model_;
    
    bool show_im_, transpose_;
    
    ros::ServiceServer depth_generation_service_;
        
    unsigned int scale_;
    
    cv::Mat image_,image_ref_;
    
    cv_bridge::CvImageConstPtr input_bridge_ref_;
    
    cv_bridge::CvImagePtr input_bridge_, output_bridge_;
    
} ;

#endif /* PIPS_COLLISION_CHECKER_H */


#ifndef RECTANGULAR_MODEL_H
#define RECTANGULAR_MODEL_H


#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv/cv.h>
#include <Eigen/Eigen>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

class HallucinatedRobotModel
{

    public: 
    virtual bool testCollision(const cv::Point3d pt)=0;
    
    void updateModel(cv::Mat& image, image_geometry::PinholeCameraModel& cam_model, double scale)
    {
      image_ref_ = image;
      cam_model_ = cam_model;
      scale_ = scale;
    }

    
    protected:

    image_geometry::PinholeCameraModel cam_model_;
    cv::Mat image_ref_;
    double robot_radius_, robot_height_, floor_tolerance_;
    double scale_;
    bool show_im_=false;




};


class RectangularModel : public HallucinatedRobotModel
{
    std::vector<cv::Point3d> co_offsets_;
    public:
    RectangularModel(double radius, double height, double safety_expansion, double floor_tolerance);
    bool testCollision(const cv::Point3d pt);
};


class CylindricalModel : public HallucinatedRobotModel
{
    public:
    CylindricalModel(double radius, double height, double safety_expansion, double floor_tolerance);
    bool testCollision(const cv::Point3d pt);

};

#endif /*RECTANGULAR_MODEL_H*/

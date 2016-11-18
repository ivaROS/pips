#ifndef RECTANGULAR_MODEL_H
#define RECTANGULAR_MODEL_H


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
    virtual bool testCollision(const cv::Point3d pt)=0;
    
    void updateModel(cv::Mat& image, image_geometry::PinholeCameraModel& cam_model, double scale)
    {
      image_ref_ = image;
      cam_model_ = cam_model;
      scale_ = scale;
      
      if(show_im_)
      {
        double min;
        double max;
        cv::minMaxIdx(image_ref_, &min, &max);
        cv::Mat adjIm;
        cv::convertScaleAbs(image_ref_, adjIm, 255 / max);
        
        cv::imshow("Original image", adjIm);
        cv::waitKey(30);
      }
    }

    
    protected:

    image_geometry::PinholeCameraModel cam_model_;
    cv::Mat image_ref_;
    double robot_radius_, robot_height_, floor_tolerance_;
    double scale_;
    bool show_im_=true;




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
    private:
    cv::Rect getColumn(const cv::Mat& image, const cv::Point2d& top, const cv::Point2d& bottom);
    public:
    CylindricalModel(double radius, double height, double safety_expansion, double floor_tolerance);
    bool testCollision(const cv::Point3d pt);

};

#endif /*RECTANGULAR_MODEL_H*/

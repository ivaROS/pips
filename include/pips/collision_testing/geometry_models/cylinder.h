#ifndef PIPS_GEOMETRY_MODELS_CYLINDER_H
#define PIPS_GEOMETRY_MODELS_CYLINDER_H

//#include <sensor_msgs/Image.h>
//#include <geometry_msgs/TransformStamped.h>

//#include <opencv/cv.h>

#include <pips/collision_testing/geometry_models/generic_models.h>
//#include <opencv2/core/core.hpp>
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"


//#include <Eigen/Eigen>
//#include <image_transport/image_transport.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <cv_bridge/cv_bridge.h>

//#include <iomanip>      // std::setprecision

#include "ros/ros.h" //Needed for 'ROS_BREAK and ROS_ASSERT



namespace pips
{
  namespace collision_testing
  {
    namespace geometry_models
    {
      

class Cylinder : public GenericGeometryModel
{
public:
    double radius_=-1, height_=-1;
    
public:
    Cylinder()
    {
      type_ = "Cylinder";
      type_id_ = 0;
    }
  
    Cylinder(double radius, double height):
      radius_(radius),
      height_(height)
    {
      type_ = "Cylinder";
      type_id_ = 0;
      
      marker_.scale.x = radius_*2;
      marker_.scale.y = radius_*2;
      marker_.scale.z = height_;

      marker_.type = visualization_msgs::Marker::CYLINDER;
    }
    
    virtual void adjust(double inflation, double tolerance)
    {
      double& z = current_transform_.transform.translation.z;
      
      height_ += 2*inflation;
      radius_ += inflation;
      
      double leeway = (z - height_/2) - tolerance;
      if( leeway < 0)
      {
        height_ += leeway;
        z -= leeway/2;
      }
      
      marker_.scale.x = radius_*2;
      marker_.scale.y = radius_*2;
      marker_.scale.z = height_;
    }

  
};

    }
  }
}
  

#endif


#ifndef PIPS_GEOMETRY_MODELS_BOX_H
#define PIPS_GEOMETRY_MODELS_BOX_H

#include <pips/collision_testing/geometry_models/generic_models.h>

namespace pips
{
  namespace collision_testing
  {
    namespace geometry_models
    {
      

class Box : public GenericGeometryModel
{
public:
    double length_ = -1, width_ = -1, height_ = -1;
    
public:
    Box()
    {
      type_ = "Box";
      type_id_ = 1;
    }
  
    Box(double length, double width, double height):
      length_(length),
      width_(width),
      height_(height)
    {
      type_ = "Box";
      type_id_ = 1;
      
      marker_.scale.x = length_;
      marker_.scale.y = width_;
      marker_.scale.z = height_;
      
      marker_.type = visualization_msgs::Marker::CUBE;
    }
    
    
    virtual void adjust(double inflation, double tolerance)
    {
      double& z = current_transform_.transform.translation.z;
      
      height_ += 2*inflation;
      width_ += 2*inflation;
      length_ += 2*inflation;
      
      double leeway = (z - height_/2) - tolerance;
      if( leeway < 0)
      {
        height_ += leeway;
        z -= leeway/2;
      }
      
      marker_.scale.x = length_;
      marker_.scale.y = width_;
      marker_.scale.z = height_;
    }

};

    }
  }
}


#endif

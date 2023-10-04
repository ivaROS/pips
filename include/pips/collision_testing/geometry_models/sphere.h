#ifndef PIPS_GEOMETRY_MODELS_SPHERE_H
#define PIPS_GEOMETRY_MODELS_SPHERE_H

#include <pips/collision_testing/geometry_models/generic_models.h>

namespace pips
{
  namespace collision_testing
  {
    namespace geometry_models
    {
      

class Sphere : public GenericGeometryModel
{
public:
    double radius_ = -1;
    
public:
    Sphere()
    {
      type_ = "Sphere";
      type_id_ = 2;
    }
  
    Sphere(double radius):
      radius_(radius)
    {
      type_ = "Sphere";
      type_id_ = 2;
      
      marker_.scale.x = 2*radius_;
      marker_.scale.y = 2*radius_;
      marker_.scale.z = 2*radius_;
      
      marker_.type = visualization_msgs::Marker::SPHERE;
    }
    
    
    virtual void adjust(double inflation, double tolerance)
    {
      //TODO
    }

};

    }
  }
}


#endif

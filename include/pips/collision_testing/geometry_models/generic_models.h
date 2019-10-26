#ifndef PIPS_GENERIC_GEOMETRY_MODELS_H
#define PIPS_GENERIC_GEOMETRY_MODELS_H

#include <string>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>

namespace pips
{
namespace collision_testing
{
namespace geometry_models
{

class GenericGeometryModel
{
  public:
    std::string frame_id_;
    std::string name_;
    std::string type_;
    urdf::Pose origin_;
    geometry_msgs::TransformStamped origin_transform_;
    geometry_msgs::TransformStamped current_transform_;
    visualization_msgs::Marker marker_;
    
    int type_id_=-1;
    
    virtual void adjust(double inflation, double tolerance) {}
    
    void setOrigin(const geometry_msgs::TransformStamped& transform)
    {
      origin_transform_ = transform;
    }
    

};

}

}

}

#endif

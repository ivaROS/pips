#ifndef PIPS_GEOMETRY_MODELS_H
#define PIPS_GEOMETRY_MODELS_H

#include <pips/utils/abstract_camera_model.h>
#include <urdf/model.h>
#include <pips/collision_testing/robot_models/column_type.h>


//this is just included to include a bunch of other stuff until I figure out exactly what is needed
#include <pips/collision_testing/robot_models/hallucinated_robot_model.h>

namespace pips
{
namespace collision_testing
{
namespace geometry_models
{

class GeometryModel
{
  private:
  
  public:
    std::string frame_id_;
    std::shared_ptr<pips::utils::AbstractCameraModel> cam_model_;
    std::string name_;
    urdf::Pose origin_;
    
    
    virtual std::vector<COLUMN_TYPE> getColumns(const geometry_msgs::Pose& pose, int img_width, int img_height)=0;

};

}

}

}

#endif
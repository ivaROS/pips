#ifndef PIPS_GEOMETRY_MODELS_H
#define PIPS_GEOMETRY_MODELS_H

#include <pips/utils/abstract_camera_model.h>
#include <urdf/model.h>
#include <pips/collision_testing/robot_models/column_type.h>

#include <visualization_msgs/Marker.h>

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
    std::string type_;
    urdf::Pose origin_;
    geometry_msgs::TransformStamped origin_transform_;
    geometry_msgs::TransformStamped current_transform_;
    visualization_msgs::Marker marker_;
    
    
    void setOrigin(const geometry_msgs::TransformStamped& transform)
    {
      origin_transform_ = transform;
    }
    
//     void setOrigin(const urdf::Pose& origin)
//     {
//       origin_ = origin;
//       geometry_msgs::TransformStamped transform;
//       
//       geometry_msgs::Vector3& t = transform.transform.translation;
//       geometry_msgs::Quaternion& rot = transform.transform.rotation;
//       
//       t.x=origin.position.x;
//       t.y=origin.position.y;
//       t.z=origin.position.z;
//       
//       origin.rotation.getQuaternion(rot.x,rot.y,rot.z,rot.w);
//       
//       origin_transform_ = transform; 
//     }
    
    virtual std::vector<COLUMN_TYPE> getColumns(const geometry_msgs::Pose& pose, int img_width, int img_height)=0;

};

}

}

}

#endif

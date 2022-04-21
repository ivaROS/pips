#ifndef PIPS_IMAGE_GEOMETRY_CONVERTER_H
#define PIPS_IMAGE_GEOMETRY_CONVERTER_H

#include <pips/collision_testing/geometry_models/geometry_models.h>
#include <pips/collision_testing/geometry_models/cylinder.h>
#include <pips/collision_testing/geometry_models/box.h>

#include <pips/collision_testing/image_geometry_models/cylinder.h>
#include <pips/collision_testing/image_geometry_models/box.h>
#include <memory>


namespace pips
{
  namespace collision_testing
  {
    namespace image_geometry_models
    {

      class ImageGeometryConverter
      {
      public:
        using geometry_type = std::shared_ptr<pips::collision_testing::image_geometry_models::GeometryModel>;
          
        static geometry_type convert(const std::shared_ptr<const pips::collision_testing::geometry_models::GenericGeometryModel>& model, const geometry_msgs::Pose& model_pose)
        {
          ROS_DEBUG_STREAM("Model name: " << model->name_ << ", type_id: " << model->type_id_);
          
          geometry_type ptr;
          
          //TODO: Move the switch to higher level class and just implement the necessary conversion functions. ex: <geometry_type, pips::collision_testing::geometry_models::Cylinder>
          switch(model->type_id_)
          {
            case 0:
              ptr = std::make_shared<pips::collision_testing::image_geometry_models::Cylinder>(std::static_pointer_cast<const pips::collision_testing::geometry_models::Cylinder>(model), model_pose);
              break;
            case 1:
              //ROS_WARN_STREAM("WARNING! Box model not currently working with ImageGeometryConverter!" );
              ptr = std::make_shared<pips::collision_testing::image_geometry_models::Box>(std::static_pointer_cast<const pips::collision_testing::geometry_models::Box>(model), model_pose);
              break;
            default:
              ROS_WARN_STREAM("WARNING! Attempted to convert a geometry model (" << model->type_id_ << ") for which no conversion is known!");
              //return nullptr;
          }
          return ptr;
        }
      };
    }
  }
} //namespace pips


#endif //PIPS_IMAGE_GEOMETRY_CONVERTER_H

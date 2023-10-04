#ifndef PIPS_GENERAL_COLLISION_CHECKER_H
#define PIPS_GENERAL_COLLISION_CHECKER_H

#include <pips/collision_testing/transforming_collision_checker.h>
#include <pips/collision_testing/robot_models/robot_model.h>

namespace pips
{
    namespace collision_testing
    {
        
        class GeneralCollisionChecker : public TransformingCollisionChecker
        {
        protected:
            geometry_msgs::TransformStamped base_optical_transform_;
            std::shared_ptr<pips::collision_testing::robot_models::RobotModel> robot_model_;
            
        public :
            
            GeneralCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, tf2_utils::TransformManager tfm=tf2_utils::TransformManager(false)) : 
              TransformingCollisionChecker(nh, pnh, name)
            {}

            virtual void setTransform(const geometry_msgs::TransformStamped& base_optical_transform)
            {
              base_optical_transform_ = base_optical_transform;
              robot_model_->setTransform(base_optical_transform);
            }
            
            virtual std_msgs::Header getCurrentHeader()
            {
              return base_optical_transform_.header;
            }
            
            virtual void initImpl()
            {
              robot_model_->init();
            }
            
        public:
          static constexpr const char* DEFAULT_NAME="abstract_general_collision_checker";
          
        } ;

    }
}


#endif /* PIPS_GENERAL_COLLISION_CHECKER_H */


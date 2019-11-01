#ifndef TRANSFORMING_COLLISION_CHECKER_H
#define TRANSFORMING_COLLISION_CHECKER_H

#include <pips/collision_testing/collision_checker.h>


#include <geometry_msgs/TransformStamped.h>


namespace pips
{
    namespace collision_testing
    {
        
        class TransformingCollisionChecker : public CollisionChecker
        {



        public :
            
            TransformingCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME) : 
                CollisionChecker(nh, pnh, name)
            {}

            virtual void setTransform(const geometry_msgs::TransformStamped& base_optical_transform)=0;
            
            static constexpr const char* DEFAULT_NAME="abstract_transforming_collision_checker";

        } ;

    }
}


#endif /* TRANSFORMING_COLLISION_CHECKER_H */


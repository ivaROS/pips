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
            
            TransformingCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string name="TransformingCollisionChecker") : 
                CollisionChecker(nh, pnh, name)
            {}
            

            
            virtual void setTransform(const geometry_msgs::TransformStamped& base_optical_transform)=0;
            


        } ;

    }
}


#endif /* TRANSFORMING_COLLISION_CHECKER_H */


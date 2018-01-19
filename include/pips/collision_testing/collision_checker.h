#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H


#include <pips/TestCollision.h>

#include <pips/utils/pose_conversions.h>

#include "ros/ros.h"
#include <memory>


#include <pips/collision_testing/collision_checking_options.h>
#include <pips/collision_testing/collision_checking_result.h>
#include <pips/utils/duration_accumulator.h>

class CollisionChecker
{


public :
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef geometry_msgs::Pose PoseType;

    CollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    CCResult testCollision(PoseType pose, CCOptions options = CCOptions());
    
    void init();

    
    template<typename T>
    CCResult testCollision(const T pose_in, CCOptions options = CCOptions())
    {
      PoseType pose_out;
      convertPose(pose_in, pose_out);
      return testCollision(pose_out, options);
    }

    /*
    template<typename T>
    auto generateVisualization(const T pose_in) -> decltype(generateVisualization(pose_out))
    {
      PoseType pose_out;
      convertPose(pose_in, pose_out);
      return generateVisualization(pose_out);
    }
    */
    
private:
   bool testCollisionSrv(pips::TestCollision::Request &req, pips::TestCollision::Response &res);
   
   virtual CCResult testCollisionImpl(PoseType pose, CCOptions options)=0;

   virtual void initImpl() {}

protected:
    //ros::Publisher posepub_;

   
private :
    std::string name_ = "CollisionChecker";
    ros::NodeHandle nh_, pnh_;	// For now, separate node handles for base and derived
    ros::ServiceServer collision_testing_service_;

    pips::utils::DurationAccumulator durations_;
                              
} ;
/*
    template<typename T, typename S>
    void getPose(T pose_in, S pose_out)
    {
      tf2::fromMsg(pose_in, pose_out);
      return pose_out;
    }
    
    template<typename T>
    PoseType getPose(T pose_in)
    {
      PoseTyp pose_out;
      getPose(pose_in, pose_out);
      return pose_out;
    }
*/


    //PoseType getPose(geometry_msgs::Point point);        
    //eigen::Affine3d getPose(geometry_msgs::Pose pose);

#endif /* COLLISION_CHECKER_H */


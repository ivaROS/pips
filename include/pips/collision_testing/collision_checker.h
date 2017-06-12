#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H


#include <pips/TestCollision.h>

#include <pips/utils/pose_conversions.h>

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <memory>


class CollisionChecker
{


public :
  typedef geometry_msgs::Pose PoseType;

    CollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    bool testCollision(PoseType pose);
    
    void init();

    
    template<typename T>
    bool testCollision(const T pose_in)
    {
      PoseType pose_out;
      convertPose(pose_in, pose_out);
      return testCollision(pose_out);
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
   
   virtual bool testCollisionImpl(PoseType pose)=0;

   virtual void initImpl() {}
    
private :
    std::string name_ = "CollisionChecker";
    ros::NodeHandle nh_, pnh_;	// For now, separate node handles for base and derived
    ros::Publisher posepub_;
    ros::ServiceServer collision_testing_service_;

                              
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


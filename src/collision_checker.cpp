#include <pips/collision_testing/collision_checker.h>
 
#include <chrono>

 
 CollisionChecker::CollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh, name_), pnh_(pnh, name_)
 {
   
 }

 
 bool CollisionChecker::testCollisionSrv(pips::TestCollision::Request &req, pips::TestCollision::Response &res)
  {
    ROS_INFO_STREAM_NAMED(name_, "Collision test request received.");
    res.collision.data = testCollision(req.pose);
    
    return true;
  }
  
  
  void CollisionChecker::init()
  {
      //TODO: is it possible for the service to be advertised and an attempt made to use it before initImpl finishes?
      collision_testing_service_ = nh_.advertiseService("test_collision", &CollisionChecker::testCollisionSrv, this);
      initImpl();
  }
  
 CCResult CollisionChecker::testCollision(geometry_msgs::Pose pose, CCOptions options)
 {
    //Start the clock
    auto t1 = std::chrono::high_resolution_clock::now();

    CCResult collided = testCollisionImpl(pose, options);
    
    //Calculate elapsed time for this computation
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
    
    ROS_DEBUG_STREAM_NAMED(name_, "Collision checking took " << fp_ms.count() << " ms");

    return collided;
 }
  
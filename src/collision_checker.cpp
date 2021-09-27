#include <pips/collision_testing/collision_checker.h>
 
#include <chrono>

 
  CollisionChecker::CollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string sub_name) : 
    nh_(nh), 
    pnh_(pnh, name_), 
    sub_name_(sub_name), 
    durations_(),
    inited_(false)
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
    if(!inited_)
    {
      //TODO: is it possible for the service to be advertised and an attempt made to use it before initImpl finishes?
      collision_testing_service_ = pnh_.advertiseService("test_collision", &CollisionChecker::testCollisionSrv, this);
      initImpl();
      inited_ = true;
    }
  }
  
 CCResult CollisionChecker::testCollision(geometry_msgs::Pose pose, CCOptions options)
 {
    //Start the clock
    //auto t1 = std::chrono::high_resolution_clock::now();
    auto t1 = ros::WallTime::now();

    CCResult collided = testCollisionImpl(pose, options);
    
    //Calculate elapsed time for this computation
    //auto t2 = std::chrono::high_resolution_clock::now();
    auto t2 = ros::WallTime::now();

    //std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
    
    //ROS_DEBUG_STREAM_NAMED(name_, "Collision checking took " << fp_ms.count() << " ms");
    //int64_t duration = (t2-t1).toNSec();
    durations_.addDuration(t1,t2);
    
    ROS_DEBUG_STREAM_NAMED(name_ + ".timing", "[" << sub_name_ << "]: Collision Checking Duration = " << durations_.getLastDuration() << ". Distance = " << pose.position.x << ". Average Duration: " << durations_.averageDuration());
    

    return collided;
 }
  

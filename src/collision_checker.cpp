#include <pips/collision_testing/collision_checker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>

 
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
 
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
      collision_pub_ = pnh_.advertise<PointCloud>("collisions",100);
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
    
    if(collided && collision_pub_.getNumSubscribers()>0)
    {
      PointCloud::Ptr msg (new PointCloud);
      std_msgs::Header header = getCurrentHeader();
      // TODO: use the pcl_conversions package instead
      msg->header.frame_id = header.frame_id;
      msg->header.stamp = header.stamp.toNSec()/1e3;	//https://answers.ros.org/question/172241/pcl-and-rostime/
      
      if(options)
      {
        auto points = collided.collision_points_;
        for(auto point : collided.collision_points_)
        {
          msg->points.push_back(point);
        }
        
        //msg->points.insert(std::end(msg->points), std::begin(points), std::end(points));
        msg->height = 1;
        msg->width = points.size();
        
      }
      else
      {
        msg->height = msg->width = 1;
        msg->points.push_back (pcl::PointXYZ(pose.position.x, pose.position.y, pose.position.z));
      }
      
      collision_pub_.publish(msg);
    }
    
    //ROS_DEBUG_STREAM_NAMED(name_, "Collision checking took " << fp_ms.count() << " ms");
    //int64_t duration = (t2-t1).toNSec();
    durations_.addDuration(t1,t2);
    
    ROS_DEBUG_STREAM_NAMED(name_ + ".timing", "[" << sub_name_ << "]: Collision Checking Duration = " << durations_.getLastDuration() << ". Distance = " << pose.position.x << ". Average Duration: " << durations_.averageDuration());
    

    return collided;
 }
 
 std_msgs::Header CollisionChecker::getCurrentHeader()
 {
   std_msgs::Header header;
   header.stamp = ros::Time::now();
   return header;
 }
 
  

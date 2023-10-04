#include <pips/collision_testing/collision_checker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>

 
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
 
  CollisionChecker::CollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name) : 
    name_(name),
    nh_(nh), 
    pnh_(pnh, name_), 
    setup_durations_(),
    cc_durations_(),
    inited_(false)
  {
    
  }

 
  bool CollisionChecker::testCollisionSrv(pips::TestCollision::Request &req, pips::TestCollision::Response &res)
  {
    ROS_INFO_STREAM_NAMED(name_ + ".collision_testing_service", "Collision test request received.");
    res.collision.data = testCollision(req.pose);
    
    return true;
  }
  
  
  void CollisionChecker::init()
  {
    if(!inited_)
    {
      initImpl();
      collision_testing_service_ = pnh_.advertiseService("test_collision", &CollisionChecker::testCollisionSrv, this);
      collision_pub_ = pnh_.advertise<PointCloud>("collisions",100);
      inited_ = true;
    }
  }
  
 CCResult CollisionChecker::testCollision(geometry_msgs::Pose pose, CCOptions options)
 {
   if(inited_)
   {
      //Start the clock
      //auto t1 = std::chrono::high_resolution_clock::now();
      auto t1 = ros::WallTime::now();

      // CCResult collided = testCollisionImpl(pose, options);

      std::vector<CCResult> all_results;
      std::vector<PoseType> multi_poses = getMultiPoses(pose, deviated_ang_, one_side_num_);
      for(auto p : multi_poses)
      {
        all_results.push_back(testCollisionImpl(p, options));
      }

      CCResult collided = mergeCCResults(all_results);
      
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
      cc_durations_.addDuration(t1,t2);
      
      ROS_DEBUG_STREAM_NAMED(name_ + ".timing", "[" << name_ << "]: Collision Checking Duration = " << cc_durations_.getLastDuration() << ". Distance = " << pose.position.x << ". Average Duration: " << cc_durations_.averageDuration());
      return collided;
   }
   else
   {
     ROS_ERROR_STREAM_NAMED(name_, "Attempting to testCollision before collision checker has been initialized!");
     return false;
   }
 }
 
 std_msgs::Header CollisionChecker::getCurrentHeader()
 {
   std_msgs::Header header;
   header.stamp = ros::Time::now();
   return header;
 }
 
  

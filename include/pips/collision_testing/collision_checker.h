#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H


#include <pips/TestCollision.h>

#include <pips/utils/pose_conversions.h>

#include <ros/ros.h>
#include <memory>


#include <pips/collision_testing/collision_checking_options.h>
#include <pips/collision_testing/collision_checking_result.h>
#include <pips/utils/duration_accumulator.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

class CollisionChecker
{


public :
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef geometry_msgs::Pose PoseType;

    CollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME);

    CCResult testCollision(PoseType pose, CCOptions options = CCOptions());
    
    //This used to be private, which made more sense from an inheritance perspective. However, I now have reason to use it directly, at least for now
    virtual CCResult testCollisionImpl(PoseType pose, CCOptions options)=0;
    
    void init();

    void setMultiPosesParams(double deviate_angle, int one_side_num)
    {
      deviated_ang_ = deviate_angle;
      one_side_num_ = one_side_num;
    }
    
    template<typename T>
    CCResult testCollision(const T pose_in, CCOptions options = CCOptions())
    {
      PoseType pose_out;
      convertPose(pose_in, pose_out);
      return testCollision(pose_out, options);
    }
    
    virtual std_msgs::Header getCurrentHeader();
    
    std::string getName() const {return name_;}

    PoseType getRotatedPose(PoseType orig_pose, double ang)
    {
      Eigen::Quaterniond q(orig_pose.orientation.w, orig_pose.orientation.x, orig_pose.orientation.y, orig_pose.orientation.z);
      auto orig_euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
      double orig_yaw = orig_euler[2];
      double new_yaw = orig_yaw + ang;
      new_yaw = new_yaw > M_PI ? (new_yaw - 2 * M_PI) : new_yaw;
      new_yaw = new_yaw < -M_PI ? (new_yaw + 2 * M_PI) : new_yaw;
      Eigen::Quaterniond new_q = Eigen::AngleAxisd(orig_euler[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(orig_euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(new_yaw, Eigen::Vector3d::UnitZ());
      PoseType new_pose = orig_pose;
      new_pose.orientation.x = new_q.x();
      new_pose.orientation.y = new_q.y();
      new_pose.orientation.z = new_q.z();
      new_pose.orientation.w = new_q.w();

      return new_pose;
    }

    std::vector<PoseType> getMultiPoses(PoseType pose, double deviate_angle, int one_side_num)
    {
      std::vector<PoseType> multi_poses;
      multi_poses.push_back(pose);
      
      if(abs(deviate_angle - M_PI) <= 1e-4)
      {
        PoseType new_pose = getRotatedPose(pose, -M_PI);
        multi_poses.push_back(new_pose);
        return multi_poses;
      }

      for(int i = 1; i <= one_side_num; i++)
      {
        PoseType l_pose = getRotatedPose(pose, i * deviate_angle);
        PoseType r_pose = getRotatedPose(pose, -i * deviate_angle);
        multi_poses.push_back(l_pose);
        multi_poses.push_back(r_pose);
      }

      return multi_poses;
    }

    CCResult mergeCCResults(std::vector<CCResult> all_results)
    {
      CCResult merged_result;
      for(auto r : all_results)
      {
        if(r)
        {
          merged_result.addPoints(r.getCollisionPnts());
        }
      }

      return merged_result;
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
public:
  static constexpr const char* DEFAULT_NAME="abstract_collision_checker";
  
private:
  bool testCollisionSrv(pips::TestCollision::Request &req, pips::TestCollision::Response &res);
  
  virtual void initImpl() {}

protected:
  std::string name_;
  ros::NodeHandle nh_, pnh_;	// For now, separate node handles for base and derived
  pips::utils::DurationAccumulator setup_durations_;

  double deviated_ang_ = 0;
  int one_side_num_ = 0;
   
private :    
    ros::ServiceServer collision_testing_service_;
    ros::Publisher collision_pub_;    
    
    pips::utils::DurationAccumulator cc_durations_;
    bool inited_;
                              
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


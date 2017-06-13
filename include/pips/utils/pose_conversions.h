#ifndef POSE_CONVERSIONS_H
#define POSE_CONVERSIONS_H

#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h> //For creating quaternion easily
#include <tf2_eigen/tf2_eigen.h> // For converting betwen Eigen and tf types

#include <opencv2/core/types.hpp>


// should really put these in some sort of namespace... and probably into a separate header

  template<typename T>
  void convertPose(const T& A, T& B)
  {
    B = A;
  }
  
  inline
  void convertPose(const geometry_msgs::Pose pose_in, cv::Point3d& pose_out)
  {
      pose_out = cv::Point3d(pose_in.position.x, pose_in.position.y, pose_in.position.z);
  }
  
  inline
  void convertPose(double pose_in[], geometry_msgs::Pose& pose_out)
  {
    pose_out.position.x = pose_in[0];
    pose_out.position.y = pose_in[1];
    pose_out.position.z = pose_in[2];
    
    if(sizeof(pose_in) / sizeof(double) == 4)
    {
      pose_out.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose_in[3]);
    }
    else
    {
      pose_out.orientation.w = 1;
    }
  }


  inline
  void convertPose(const Eigen::Affine3d& in, geometry_msgs::Pose& msg) {
    msg = tf2::toMsg(in);
    // Older versions of tf2_eigen didn't include the above function, requiring the below code:
    /*  
     msg.position.x = in.translation().x();
     msg.position.y = in.translation().y();
     msg.position.z = in.translation().z();
     msg.orientation.x = Eigen::Quaterniond(in.rotation()).x();
     msg.orientation.y = Eigen::Quaterniond(in.rotation()).y();
     msg.orientation.z = Eigen::Quaterniond(in.rotation()).z();
     msg.orientation.w = Eigen::Quaterniond(in.rotation()).w(); 
     */
  }
  
  /* Note: not sure whether this one actually works */
  inline
  void convertPose(const geometry_msgs::Pose& msg, Eigen::Affine3d& out)
  {
    tf2::fromMsg(msg, out);
  }
  
  inline
  void convertPose(const geometry_msgs::Pose& msg, geometry_msgs::Pose& out)
  {
    out = msg;
  }
  
  std::string toString(const geometry_msgs::Pose& pose)
  {
    std::stringstream ss;
    ss << "[" << pose.position.x << "," << pose.position.y << "," << pose.position.z << "] (" <<
        pose.orientation.w << "," << pose.orientation.x << "," << pose.orientation.y << "," << pose.orientation.z << 
        ")";
    return ss.str();
  }
  
  std::string toString(const geometry_msgs::TransformStamped& transform)
  {
    std::stringstream ss;
    ss << "[" << transform.transform.translation.x << "," << transform.transform.translation.y << "," << transform.transform.translation.z << "] (" <<
        transform.transform.rotation.w << "," << transform.transform.rotation.x << "," << transform.transform.rotation.y << "," << transform.transform.rotation.z << 
        ")";
    return ss.str();
  }
  
#endif // POSE_CONVERSIONS_H
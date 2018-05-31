#ifndef COLLISION_CHECKING_RESULT_H
#define COLLISION_CHECKING_RESULT_H


#include <opencv2/core/types.hpp>
//#include <pcl/point_types.h>


template <typename T>
struct CollisionPoint_
{
  T x,y,z;
  
  template <typename S>
  CollisionPoint_(const cv::Point3_<S> pt)
  {
    x = pt.x;
    y = pt.y;
    z = pt.z;
  }
  
  CollisionPoint_(const pcl::_PointXYZ pt)
  {
    x = pt.x;
    y = pt.y;
    z = pt.z;
  }
  
  operator pcl::PointXYZ() const 
  {
    return pcl::PointXYZ(x, y, z);
  }
  
//   template <typename S>
//   operator cv::Point3_<S>() const 
//   {
//     return cv::Point3_<S>(x, y, z);
//   }
  
  operator cv::Point3_<float>() const 
  {
    return cv::Point3_<float>(x, y, z);
  }
  
  operator cv::Point3_<double>() const 
  {
    return cv::Point3_<double>(x, y, z);
  }
  
};

typedef CollisionPoint_<float> CollisionPoint;

struct CCResult
{
  bool collides_=false;
  bool has_details_=false;

  
  std::vector<CollisionPoint> collision_points_;
  
  
  CCResult() {}
  
  CCResult(bool is_collision) :
      collides_(is_collision) 
      {}
  
  CCResult(CollisionPoint collision_point) : 
      collides_(true),
      has_details_(true)
      {
	collision_points_.push_back(collision_point);
      }
  
  CCResult(std::vector<CollisionPoint> collision_points) : 
    collides_(true),
    has_details_(true),
    collision_points_(collision_points)
    {
    }
  
  
  operator bool() const
  {
      return collides(); // Or false!
  }
  
  bool collides() const
  {
    return collides_;
  }
  
  std::vector<CollisionPoint> getCollisionPnts() const
  {
    return collision_points_;
  }
  
  bool details() const
  {
    return has_details_;
  }
};

#endif /* COLLISION_CHECKING_RESULT_H */

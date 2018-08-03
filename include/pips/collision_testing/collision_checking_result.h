#ifndef COLLISION_CHECKING_RESULT_H
#define COLLISION_CHECKING_RESULT_H


#include <opencv2/core/types.hpp> //TODO: I believe that this is now unnecessary
//#include <pcl/point_types.h>


namespace pips
{
  namespace utils
  {

    template <typename S, typename T>
    void convertPoint(const S pt, T& x, T& y, T& z)
    {
      x = pt.x;
      y = pt.y;
      z = pt.z;
    }

  }
  
}

template <typename T>
struct CollisionPoint_
{
  T x,y,z;
  
  template <typename S>
  CollisionPoint_(const S pt)
  {
   pips::utils::convertPoint(pt, x, y, z);
  }
  

  
  template <typename S>
  operator S() const
  {
    return S(x,y,z);
  }
  
  /*
  operator pcl::PointXYZ() const 
  {
    return pcl::PointXYZ(x, y, z);
  }*/
  
//   template <typename S>
//   operator cv::Point3_<S>() const 
//   {
//     return cv::Point3_<S>(x, y, z);
//   }
  
//   operator cv::Point3_<float>() const 
//   {
//     return cv::Point3_<float>(x, y, z);
//   }
//   
//   operator cv::Point3_<double>() const 
//   {
//     return cv::Point3_<double>(x, y, z);
//   }
//   
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
  
  CCResult(CollisionPoint collision_point)
  {
    addPoint(collision_point);
  }
  
  CCResult(std::vector<CollisionPoint> collision_points) : 
    collides_(true),
    has_details_(true),
    collision_points_(collision_points)
  {
  }
  
  template <typename T>
  CCResult(T result);
  
  void addPoint(CollisionPoint collision_point)
  {
    collides_ = true;
    has_details_ = true;
    collision_points_.push_back(collision_point);
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

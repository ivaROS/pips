#ifndef COLLISION_CHECKING_RESULT_H
#define COLLISION_CHECKING_RESULT_H


//#include <opencv2/core/types.hpp> //TODO: I believe that this is now unnecessary
//#include <pcl/point_types.h>

//TODO: Move all of this to a sensible namespace



template <typename T>
struct CollisionPoint_
{
  T x,y,z;
  
//   template <typename S>
//   CollisionPoint_(const S pt)
//   {
//    pips::utils::convertPoint(pt, x, y, z);
//   }
  
  CollisionPoint_(T x, T y, T z) :
    x(x), y(y), z(z)
    {}
  

  
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



namespace pips
{
  namespace collision_testing
  {
    typedef CollisionPoint_<float> CollisionPoint;
    
    
//template <typename S, typename T>
//void convertPoint(const S pt, T& x, T& y, T& z);

    template <typename S>
    CollisionPoint toCollisionPoint(const S pt)
    {
      CollisionPoint co(pt.x,pt.y,pt.z);
      return co;
    }

  }

//   namespace collision_testing
//   {
//     template <typename T, typename U>
//     std::vector<U> getPoints(const T& result);
// //     {
// //       std::vector<U> points;
// //       return points;
// //     }
//   }

}



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
  
//   template <typename T, typename U>
//   CCResult(T result)
//   {
//     std::vector<U> points = pips::collision_testing::getPoints(result);
//     for(U point : points)
//     {
//       addPoint(point);
//     }
//   }
  
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

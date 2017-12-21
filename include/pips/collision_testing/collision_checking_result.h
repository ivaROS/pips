#ifndef COLLISION_CHECKING_RESULT_H
#define COLLISION_CHECKING_RESULT_H


#include <opencv2/core/types.hpp>

struct CCResult
{
  bool collides_=false;
  bool has_details_=false;
  std::vector<cv::Point3d> collision_points_;
  
  
  CCResult() {}
  
  CCResult(bool is_collision) :
      collides_(is_collision) 
      {}
  
  CCResult(cv::Point3d collision_point) : 
      collides_(true),
      has_details_(true)
      {
	collision_points_.push_back(collision_point);
      }
  
  CCResult(std::vector<cv::Point3d> collision_points) : 
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
  
  std::vector<cv::Point3d> getCollisionPnts() const
  {
    return collision_points_;
  }
  
  bool details() const
  {
    return has_details_;
  }
};

#endif /* COLLISION_CHECKING_RESULT_H */
#ifndef COLLISION_CHECKING_RESULT_H
#define COLLISION_CHECKING_RESULT_H


#include <opencv2/core/types.hpp>

struct CCResult
{
  bool collides_=false;
  bool has_details_=false;
  cv::Point3d collision_point_;
  
  
  CCResult() {}
  
  CCResult(bool is_collision) :
      collides_(is_collision) 
      {}
  
  CCResult(cv::Point3d collision_point) : 
      collides_(true),
      has_details_(true),
      collision_point_(collision_point)
      {}
  
  
  operator bool() const
  {
      return collides(); // Or false!
  }
  
  bool collides() const
  {
    return collides_;
  }
  
  cv::Point3d getCollisionPnt() const
  {
    return collision_point_;
  }
  
  bool details() const
  {
    return has_details_;
  }
};

#endif /* COLLISION_CHECKING_RESULT_H */
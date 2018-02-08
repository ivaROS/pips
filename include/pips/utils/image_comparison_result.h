#ifndef IMAGE_COMPARISON_RESULT_H
#define IMAGE_COMPARISON_RESULT_H


#include <opencv2/core/types.hpp>

struct ComparisonResult
{
  bool has_details_=false;
  bool collides_ = false;
  cv::Point collision_point_;
  float collision_depth_=-1;//nanf;
  
  ComparisonResult(bool is_collision) : collides_(is_collision) {}
  
  ComparisonResult(int row, int col, float depth) : 
      has_details_(true),
      collides_(true),
      collision_point_(col,row),
      collision_depth_(depth)
      {}
  
  ComparisonResult(const cv::Point& pt, float depth) :
    has_details_(true),
    collides_(true),
    collision_point_(pt),
    collision_depth_(depth)
    {}
  
  operator bool() const
  {
      return collides(); // Or false!
  }
  
  bool collides() const
  {
    return collides_;
  }
  
  bool has_details() const
  {
    return has_details_;
  }
  
  cv::Point point() const
  {
    return collision_point_;
  }
  
  float depth() const
  {
    return collision_depth_;
  }
};

#endif /* IMAGE_COMPARISON_RESULT_H */
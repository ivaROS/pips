#ifndef IMAGE_COMPARISON_RESULT_H
#define IMAGE_COMPARISON_RESULT_H


#include <opencv2/core/types.hpp>

struct PixelCollision
{
  cv::Point pt;
  float depth;
  
  PixelCollision(cv::Point pt, float depth):
    depth(depth),
    pt(pt)
  {}
    
  PixelCollision(int row, int col, float depth):
    depth(depth),
    pt(cv::Point(col,row))
  {}
  
};

struct ComparisonResult
{
private:
  bool has_details_=false;
  bool collides_ = false;
  std::vector<PixelCollision> collision_points_;
  
public:
  ComparisonResult()
  {}
  
  ComparisonResult(bool is_collision) : collides_(is_collision) {}
  
  ComparisonResult(int row, int col, float depth)
  {
    addPoint(row,col,depth);
  }
  
  ComparisonResult(const cv::Point& pt, float depth) :
    has_details_(true),
    collides_(true)
  {
    addPoint(pt, depth);
  }
  

  
  void addPoint(const cv::Point& pt, float depth)
  {
    PixelCollision pc(pt,depth);
    addPoint(pc);
  }
  
  void addPoint(int row, int col, float depth)
  {
    PixelCollision pc(row,col,depth);
    addPoint(pc);
  }
  
  void addPoint(PixelCollision pc)
  {
    collision_points_.push_back(pc);
    has_details_ = true;
    collides_ = true;
  }
  
  void addOffset(cv::Point pt)
  {
    for(auto& point : collision_points_)
    {
      point.pt += pt;
    }
  }
  
  void transpose()
  {
    for(auto& point : collision_points_)
    {
      cv::Point pt_t(point.pt.y, point.pt.x);
      point.pt = pt_t;
    }
  }
  
  // Getters
  bool hasDetails() const
  {
    return has_details_;
  }
  
  operator bool() const
  {
    return collides(); // Or false!
  }
  
  bool collides() const
  {
    return collides_;
  }
  
  std::vector<PixelCollision> points() const
  {
    return collision_points_;
  }

};

#endif /* IMAGE_COMPARISON_RESULT_H */
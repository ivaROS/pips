#ifndef CYLINDRICAL_MODEL_H
#define CYLINDRICAL_MODEL_H

#include <pips/collision_testing/robot_models/hallucinated_robot_model.h>
#include <pips/collision_testing/robot_models/column_type.h>
#include <pips/collision_testing/geometry_models/cylinder.h>

// May be able to template this for 32F or 16U and have the other functions templated based on the type of this that is passed in
// struct COLUMN_TYPE
// {
//   cv::Rect rect;
//   //cv::Mat image;
//   float depth;
// };

class CylindricalModel : public HallucinatedRobotModelImpl<cv::Point3d>
{
  private:

    
  public:
  
    CylindricalModel();
    
  
    void setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im);
    virtual bool inFrame(const cv::Point3d& pt);

  protected:
  
    // Should use my custom comparison code from RectangularModel, make it more general and move it somewhere (maybe a header?), then use it from both of these. Except, that built-in vectorized/parallel versions should use this...
  
    // Question: is it better to use the camera model's methods for clarity, or to use pure Eigen matrices for speed?
    // Initially, will use the model, but will likely switch over in the future.
    virtual ComparisonResult testCollisionImpl(const cv::Point3d pt, CCOptions options);
    
    virtual ComparisonResult isLessThan(const cv::Mat& col, float depth);
    virtual ComparisonResult isLessThanDetails(const cv::Mat& col, float depth);

    virtual geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose);
    
    std::vector<COLUMN_TYPE> getColumns(const cv::Point3d pt);
    
    virtual cv::Mat generateHallucinatedRobotImpl(const cv::Point3d pt);
    
    virtual cv::Rect getColumnRect(int x, int y, int width, int height);
    virtual cv::Rect getColumnRect(const cv::Rect& rect);
    
    virtual cv::Rect getROIImpl(const cv::Point3d pt);
    
    void getIntersection(cv::Point3d pt, double r0, cv::Point3d& left, cv::Point3d& right);
    
    pips::collision_testing::geometry_models::Cylinder cylinder_;
};


#endif /*  CYLINDRICAL_MODEL_H */

#ifndef CYLINDRICAL_MODEL_H
#define CYLINDRICAL_MODEL_H

#include <pips/collision_testing/robot_models/hallucinated_robot_model.h>

// May be able to template this for 32F or 16U and have the other functions templated based on the type of this that is passed in
struct COLUMN_TYPE
{
  cv::Rect rect;
  //cv::Mat image;
  float depth;
};

template<typename T>
class CylindricalModel : public HallucinatedRobotModelImpl<cv::Point3d, T>
{
  private:

    
  public:
  
    CylindricalModel() : HallucinatedRobotModelImpl<cv::Point3d,T>() 
    {
      this->name_ = "CylindricalModel";
    }
    
  
    void setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im);

  protected:
  
    // Should use my custom comparison code from RectangularModel, make it more general and move it somewhere (maybe a header?), then use it from both of these. Except, that built-in vectorized/parallel versions should use this...
  
    // Question: is it better to use the camera model's methods for clarity, or to use pure Eigen matrices for speed?
    // Initially, will use the model, but will likely switch over in the future.
    virtual bool testCollisionImpl(const cv::Point3d pt)
    {
      std::vector<COLUMN_TYPE> cols = getColumns(pt);
    
      for(unsigned int i = 0; i < cols.size(); ++i)
      {
        T col = cv::Mat(this->image_ref_,cols[i].rect); //cols[i].image;
        float depth = cols[i].depth;
        
        if(isLessThan(col, depth))
        {
          return true; 
        }
      }
   
      return false;
    }
    
    virtual bool issLessThan(T col, float depth)
    {
      return cv::countNonZero(col < depth) > 0;
    }
    
  
    
    virtual COLUMN_TYPE getColumn(const cv::Point2d top, const cv::Point2d bottom, const float depth);
    std::vector<COLUMN_TYPE> getColumns(const cv::Point3d pt);
    
    virtual cv::Mat generateHallucinatedRobotImpl(const cv::Point3d pt);
    
    virtual cv::Rect getROIImpl(int x, int y, int width, int height);

};


#endif /*  CYLINDRICAL_MODEL_H */

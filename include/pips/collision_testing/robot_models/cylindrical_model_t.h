#ifndef CYLINDRICAL_MODEL_T_H
#define CYLINDRICAL_MODEL_T_H

#include <pips/collision_testing/robot_models/cylindrical_model.h>

template <typename T>
class CylindricalModelT : public CylindricalModel<T>
{
  public:

    typedef CylindricalModel<T> super;
  
    CylindricalModelT() : CylindricalModel<T>()
    {
      this->name_ = "CylindricalModelT";
    }
    

    
  protected:
    
  cv::Mat generateHallucinatedRobotImpl(const cv::Point3d pt)
  {
    cv::Mat viz = super::generateHallucinatedRobotImpl(pt);

    cv::Mat viz_t = viz.t();
  
    return viz_t;
  }

    
    virtual cv::Rect getROIImpl(int x, int y, int width, int height);

    virtual T getImageImpl(const T& image)
    { 
      T transposed = image.t();
      return transposed;
    }

};


#endif /*  CYLINDRICAL_MODEL_T_H */

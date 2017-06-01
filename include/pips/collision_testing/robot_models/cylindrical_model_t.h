#ifndef CYLINDRICAL_MODEL_T_H
#define CYLINDRICAL_MODEL_T_H

#include <pips/collision_testing/robot_models/cylindrical_model.h>

class CylindricalModelT : public CylindricalModel
{
  public:
  
    CylindricalModelT();
    
  protected:
    
    virtual cv::Mat generateHallucinatedRobotImpl(const cv::Point3d pt);
    
    virtual cv::Rect getColumnRect(int x, int y, int width, int height);

    virtual cv::Mat getImageImpl(const cv::Mat& image);

};


#endif /*  CYLINDRICAL_MODEL_T_H */

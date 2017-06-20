#ifndef RECTANGULAR_MODEL_OCL
#define RECTANGULAR_MODEL_OCL

#include <pips/collision_testing/robot_models/rectangular_model.h>
#include <opencv2/core/ocl.hpp>

class RectangularModelOCL : public RectangularModel
{
  public:
    RectangularModelOCL();
    
  protected:  
    cv::UMat image_cl_;
    
  protected:
    virtual void doPrecomputation();

    virtual bool testCollisionImpl(const cv::Point3d pt);
    
    

    
};

#endif // RECTANGULAR_MODEL_OCL

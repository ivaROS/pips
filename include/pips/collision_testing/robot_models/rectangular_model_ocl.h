#ifndef RECTANGULAR_MODEL_OCL
#define RECTANGULAR_MODEL_OCL

#include <pips/collision_testing/robot_models/rectangular_model.h>
#include <opencv2/core/ocl.hpp>
#include <boost/thread/mutex.hpp>

class RectangularModelOCL : public RectangularModel
{
  public:
    RectangularModelOCL();
    
  protected:  
    cv::UMat image_cl_, roi_cl_, res_cl_;
    boost::mutex model_mutex_;
    
  protected:
    virtual void doPrecomputation(const cv_bridge::CvImage::ConstPtr& cv_image_ref);

    virtual bool testCollisionImpl(const cv::Point3d pt);
    
    

    
};

#endif // RECTANGULAR_MODEL_OCL

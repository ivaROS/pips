#ifndef CYLINDRICAL_MODEL_T_VECT
#define CYLINDRICAL_MODEL_T_VECT

#include <pips/collision_testing/robot_models/cylindrical_model_t.h>


class CylindricalModelTVect : public CylindricalModelT
{
  public:
    CylindricalModelTVect();
    
  protected:

    virtual ComparisonResult isLessThan(const cv::Mat& image, const float depth);
    
};

#endif // CYLINDRICAL_MODEL_T_VECT

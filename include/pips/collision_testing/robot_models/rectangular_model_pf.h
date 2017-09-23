#ifndef RECTANGULAR_MODEL_PF
#define RECTANGULAR_MODEL_PF

#include <pips/collision_testing/robot_models/rectangular_model.h>


class RectangularModelPF : public RectangularModel
{
  public:
    RectangularModelPF();
    
  protected:

    virtual bool isLessThan(const cv::Mat& image, const float depth);
    
};

#endif // RECTANGULAR_MODEL_PF

#ifndef RECTANGULAR_MODEL_SS
#define RECTANGULAR_MODEL_SS

#include <pips/collision_testing/robot_models/rectangular_model.h>


class RectangularModelSS : public RectangularModel
{
  public:
    RectangularModelSS();
    
  protected:

    virtual bool isLessThan(const cv::Mat& image, const float depth);
    

    template<typename T>
    inline
    bool isLessThan(const cv::Mat& image, const float depth)
    {
      int nRows = image.rows;
      int nCols = image.cols;
      
      int i,j;
      const T* p;
      for( i = 0; i < nRows; ++i)
      {
        p = image.ptr<T>(i);
        for ( j = 0; j < nCols; ++j)
        {
            if(p[j] < depth)
            {
              return true;
            }
              
        }
      }
      return false;
    }
};

#endif // RECTANGULAR_MODEL_SS

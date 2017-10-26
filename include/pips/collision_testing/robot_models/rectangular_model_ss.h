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
    bool isLessThan(const cv::Mat& image, const T depth)
    {
      int nRows = image.rows;
      int nCols = image.cols;
      
      bool abort = false;
      
      //#pragma omp parallel for
      for( int i = 0; i < nRows; ++i)
      {
	
	  const T* p = image.ptr<T>(i);
	  uint8_t sum = 0;
	  uint8_t temp[nCols];
	  for(int j=0; j < nCols; ++j)
	  {
	      temp[j] = (p[j] < depth) ? 1 : 0;
	      sum += temp[j];
	  }
	  
	  if( sum >0 )
	  {
            //std::cout << "sum: " << sum;
	    return true;
	  }
	
      }
      return false;
    }
    

    
    
};

#endif // RECTANGULAR_MODEL_SS

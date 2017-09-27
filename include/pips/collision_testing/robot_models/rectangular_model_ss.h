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
      
      int i;
      
      const T* p;
      for( i = 0; i < nRows; ++i)
      {
	int j = 0;
        p = image.ptr<T>(i);
	int sum = 0;
	short int temp[nCols];
	for(j=0; j < nCols; ++j)
	{
	    temp[j] = (p[j] < depth) ? 1 : 0;
	}
	for(j=0; j<nCols; ++j)
	{
	  	    sum += temp[j];
	}

	
        if( sum > 0)
	{
	  return true;
	}
      }
      return false;
    }
    
    
    /*
    template<typename T>
    inline
    bool isLessThan(const cv::Mat& image, const float depth)
    {
      int nRows = image.rows;
      int nCols = image.cols;
      
      int i;
      
      const T* p;
      for( i = 0; i < nRows; ++i)
      {
	int j = 0;
        p = image.ptr<T>(i);
	while((p[j] < depth) && (j < nCols))
	{
	    ++j;
	}
        if( j < nCols)
	{
	  return true;
	}
      }
      return false;
    }
    */
    
};

#endif // RECTANGULAR_MODEL_SS

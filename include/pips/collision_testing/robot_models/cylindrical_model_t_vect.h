#ifndef CYLINDRICAL_MODEL_VECT
#define CYLINDRICAL_MODEL_VECT

#include <pips/collision_testing/robot_models/cylindrical_model_t.h>


class CylindricalModelVect : public CylindricalModelT
{
  public:
    CylindricalModelVect();
    
  protected:

    virtual bool isLessThan(const cv::Mat& image, const float depth);
    

    template<typename T>
    inline
    bool isLessThan(const cv::Mat& image, const T depth)
    {
      int nRows = image.rows;
      int nCols = image.cols;
      
      int i;
      
      const T* p;
      for( i = 0; i < nRows; ++i)
      {
	int j = 0;
        p = image.ptr<T>(i);
	uint8_t sum = false;
	uint8_t temp[nCols];
	for(j=0; j < nCols; ++j)
	{
	    temp[j] = (p[j] < depth) ? 1 : 0;
	    sum |= temp[j];
	}

	
        if( sum )
	{
	  return true;
	}
      }
      return false;
    }
    
    /*
     *     template<typename T>
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

#endif // CYLINDRICAL_MODEL_VECT

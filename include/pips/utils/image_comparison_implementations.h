#ifndef IMAGE_COMPARISON_IMPLEMENTATIONS
#define IMAGE_COMPARISON_IMPLEMENTATIONS

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace utils
{
  namespace comparisons
  {
 
      template<typename T>
      inline
      bool simple(const cv::Mat& image, const T depth)
      {
	int nRows = image.rows;
	int nCols = image.cols;
    
	const T* p;
	for(int i = 0; i < nRows; ++i)
	{
	  p = image.ptr<T>(i);
	  for(int j = 0; j < nCols; ++j)
	  {
	    if(p[j] < depth)
	    {
	      return true;
	    }
	  }
	}
	return false;
      }
      
      template<typename T>
      inline
      /* Vectorizeable by gcc when using -ffast-math */
      bool vectorized(const cv::Mat& image, const T depth)
      {
	int nRows = image.rows;
	int nCols = image.cols;
	      
	for( int i = 0; i < nRows; ++i)
	{
	  
	    const T* p = image.ptr<T>(i);
	    uint8_t sum = 0;
	    uint8_t temp[nCols];
	    for(int j=0; j < nCols; ++j)
	    {
		temp[j] = (p[j] < depth) ? 1 : 0;
		sum |= temp[j];
	    }
	    
	    if( sum >0 )
	    {
	      return true;
	    }
	  
	}
	return false;
      }
      
  }
    
  namespace isLessThan
  {
    
    /* Returns true if collision happens (eg. the world is closer than the requested depth) */
    inline
    bool simple(const cv::Mat& image, const float depth)
    {
      if(image.depth() == CV_32FC1)
      {
	return utils::comparisons::simple<float>(image, depth);
      }
      else if (image.depth() == CV_16UC1)
      {
	return utils::comparisons::simple<uint16_t>(image, depth);
      }
      return false;
    }
    
    /* Returns true if collision happens (eg. the world is closer than the requested depth) */
    inline
    bool vectorized(const cv::Mat& image, const float depth)
    {
      if(image.depth() == CV_32FC1)
      {
	return utils::comparisons::vectorized<float>(image, depth);
      }
      else if (image.depth() == CV_16UC1)
      {
	return utils::comparisons::vectorized<uint16_t>(image, depth);
      }
      return false;
    }

    inline
    bool stock(const cv::Mat& image, const float depth)
    {
      cv::Mat res;
      cv::compare(image, depth, res, cv::CMP_LT);

      int num_collisions = cv::countNonZero(res);
      bool collided = (num_collisions > 0);
      return collided;
    }
    

    
  }
    
}

#endif //IMAGE_COMPARISON_IMPLEMENTATIONS
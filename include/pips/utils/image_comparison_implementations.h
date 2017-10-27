#ifndef IMAGE_COMPARISON_IMPLEMENTATIONS
#define IMAGE_COMPARISON_IMPLEMENTATIONS

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <type_traits>

namespace utils
{
  class comparisons
  {
    
    enum struct COMPARISON_TYPE {stock, simple, vectorized, parallel};

    
    /* Returns true if collision happens (eg. the world is closer than the requested depth) */
    template<typename V>
    inline typename std::enable_if<(not std::is_same<V, utils::comparisons::stock >::value), void>::type
    bool isLessThan(const cv::Mat& image, const float depth)
    {
      if(image.depth() == CV_32FC1)
      {
	return isLessThan<float, V>(image, depth);
      }
      else if (image.depth() == CV_16UC1)
      {
	return isLessThan<uint16_t, V>(image, depth);
      }
      return false;
    }

    template<typename V>
    inline typename std::enable_if<std::is_same<V, utils::comparisons::stock >::value, void>::type
    bool isLessThan(const cv::Mat& image, const float depth)
    {
      cv::Mat res;
      cv::compare(image, depth, res, cv::CMP_LT);

      int num_collisions = cv::countNonZero(res);
      bool collided = (num_collisions > 0);
      return collided;
    }
    
    template<typename T, typename V>
    inline
    bool isLessThan(const cv::Mat& image, const T depth);
    
    
    template<typename T, typename V>
    inline typename std::enable_if<std::is_same<V, utils::comparisons::simple >::value, void>::type
    bool isLessThan(const cv::Mat& image, const T depth)
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
    
    template<typename T, typename V>
    inline typename std::enable_if<std::is_same<V, utils::comparisons::vectorized >::value, void>::type
    /* Vectorizeable by gcc when using -ffast-math */
    bool isLessThan(const cv::Mat& image, const T depth)
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
	      sum += temp[j];
	  }
	  
	  if( sum >0 )
	  {
	    return true;
	  }
	
      }
      return false;
    }
    

    
  }
    
}

#endif //IMAGE_COMPARISON_IMPLEMENTATIONS
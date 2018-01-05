#ifndef IMAGE_COMPARISON_IMPLEMENTATIONS
#define IMAGE_COMPARISON_IMPLEMENTATIONS

#include <pips/utils/image_comparison_result.h>

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
      
    template<typename T>
    inline
    ComparisonResult details(const cv::Mat& image, const T depth)
    {
      int nRows = image.rows;
      int nCols = image.cols;
      
      int i;
      
      const T* p;
      for( i = 0; i < nRows; ++i)
      {
        p = image.ptr<T>(i);
	for(int j = 0; j < nCols; ++j)
	{
	  T pixel_depth = p[j];
	  if(pixel_depth < depth)
	  {
	    //ROS_INFO_STREAM("p: " << p[j] << ", depth: " << depth << ", i: " << i << ", j: " << j);
	    ;
	    //pnt.x = j;
	    //pnt.y = i;
	    return ComparisonResult(i,j, pixel_depth);
	  }
	}
      }
      return ComparisonResult(false);
    }
      
  }
    
    
    /* TODO: I should be able to write 1 templated version with the data type evaluations
     * and then each version can simple call it with the desired comparison function as template parameter 
     *
     * TODO: These accessor functions should be placed in a .cpp file with a separate header so that they
     * can be linked to rather than having to rebuild every time
     */
    
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
    ComparisonResult stock(const cv::Mat& image, const float depth)
    {
      cv::Mat res;
      cv::compare(image, depth, res, cv::CMP_LT);

      int num_collisions = cv::countNonZero(res);
      bool collided = (num_collisions > 0);
      
      ComparisonResult result(collided);
      
      return collided;
    }
    
    inline
    ComparisonResult details(const cv::Mat& image, const float depth)
    {
      if(image.depth() == CV_32FC1)
      {
	return utils::comparisons::details<float>(image, depth);
      }
      else if (image.depth() == CV_16UC1)
      {
	return utils::comparisons::details<uint16_t>(image, depth);
      }
      
    }
    
  }
    
}

#endif //IMAGE_COMPARISON_IMPLEMENTATIONS

#include <fstream>
#include <memory>
#include <chrono>

#include <omp.h>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

//#include <pips/utils/image_comparison_implementations.h>

      template<typename T>
      inline
uint16_t inner_loop(const cv::Mat& image, const T& depth)
{

	int nRows = image.rows;
	int nCols = image.cols;
	      
	uint16_t sum = 0;
	for( int i = 0; i < nRows; ++i)
	{
	  
	    const T* p = image.ptr<T>(i);
	    uint16_t intsum = 0;
	    uint8_t temp[nCols];
	    for(int j=0; j < nCols; ++j)
	    {
		temp[j] = (p[j] < depth) ? 1 : 0;
		intsum += temp[j];
	    }
	    sum += intsum;

	}

 
 
	    	return sum;

}


template<typename T>
      inline
/* Vectorizeable by gcc when using -ffast-math */
bool vect2(const cv::Mat& image, const T depth, const int vec_size)
{
  int nRows = image.rows;
  int nCols = image.cols;
	  
  if (image.isContinuous())
  {
      nCols *= nRows;
      nRows = 1;
  }
  
  uint main_loops = nCols / vec_size;
  uint remainder = nCols % vec_size;
  
  
  for( int i = 0; i < nRows; ++i)
  {
  
    const T* p = image.ptr<T>(i);

    
    for(int j=0; j <main_loops; ++j)
    {
      uint8_t temp[vec_size];
      uint8_t sum=0;

      for(int k=0; k < vec_size; ++k)
      {
	int a = j*vec_size + k;
	temp[k] = (p[a] < depth) ? 1 : 0;
	sum |= 0;
      }
      
      if( sum >0 )
      {
	return true;
      }
  
      
      
      
    }
    
    for(int k = 0; k < remainder; ++k)
    {
      int a = main_loops*vec_size + k; 
      uint8_t temp = (p[a] < depth) ? 1 : 0;
      if(temp > 0)
      {
	return true;
      }
    }

    
  }
  
  return false;

}  





      template<typename T>
      inline
size_t middle_loop(const cv::Mat& img, const T& depth, const int vect_size)
{
  size_t sum = 0;
  
  const uint16_t num_rows = img.rows;
  const uint16_t num_reduced_rows = img.rows / 4;
  const uint16_t num_cols = img.cols;
  
  auto t1 = std::chrono::high_resolution_clock::now();

  for(int i = 0; i < 100; ++i)
  {
    uint16_t x,y,width,height;
    
    x=0;
    
    y = i % num_reduced_rows;
    width = num_cols;
    
    height = num_rows - y;
    
    cv::Rect rect(x,y,width,height);
    cv::Mat roi(img,rect);
    uint16_t val = vect2(img, depth, vect_size);
    sum += val;
  }
  
  auto t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
  
  if(omp_in_parallel())
  {
  int thread_id = omp_get_thread_num();
  //std::cout << "OpenMP active! Thread # " << thread_id << ", loop completed in " << fp_ms.count() << "ms, count=" << sum << std::endl;

  }
  else
  {
  //std::cout << "Loop completed in " << fp_ms.count() << "ms, count=" << sum << std::endl;

  }
  
  return sum;
}



size_t parallel_outer_loop(const cv::Mat& img, uint8_t num_it, int vect_size)
{
 
    std::vector<size_t> results(num_it); 

	#pragma omp parallel for schedule(dynamic) //if(parallelism_enabled_) //schedule(dynamic)
        for(uint8_t i = 0; i < num_it; i++)
        {
	  float depth = i;
	  size_t result = middle_loop(img, depth,vect_size);
	  results[i] = result;
        }
        
        size_t sum = 0;
        for(uint8_t i = 0; i < num_it; i++)
        {
	    sum+= results[i];
        }
   
  return sum;
}


size_t outer_loop(const cv::Mat& img, uint8_t num_it, int vect_size)
{
  

    std::vector<size_t> results(num_it); //std::vector<boost::shared_ptr<PipsTrajectory*>>

        for(uint8_t i = 0; i < num_it; i++)
        {
	  float depth = i;
	  size_t result = middle_loop(img, depth, vect_size);
	  results[i] = result;
        }
        
        size_t sum = 0;
        for(uint8_t i = 0; i < num_it; i++)
        {
	    sum+= results[i];
        }
        

  return sum;
}


size_t parallel_outer_loop(const cv::Mat& img, uint8_t num_it, int vect_size, bool parallelism_enabled)
{
 
    std::vector<size_t> results(num_it); 

	#pragma omp parallel for schedule(dynamic) if(parallelism_enabled) //schedule(dynamic)
        for(uint8_t i = 0; i < num_it; i++)
        {
	  float depth = i;
	  size_t result = middle_loop(img, depth, vect_size);
	  results[i] = result;
        }
        
        size_t sum = 0;
        for(uint8_t i = 0; i < num_it; i++)
        {
	    sum+= results[i];
        }
   
  return sum;
}


bool run_comparison(const cv::Mat& img, uint8_t num_it, int vect_size)
{
      auto t1 = std::chrono::high_resolution_clock::now();

      size_t val1 = outer_loop(img,  num_it, vect_size);
  
      auto t2 = std::chrono::high_resolution_clock::now();
      
      size_t val2 = parallel_outer_loop(img,  num_it, vect_size);

      auto t3 = std::chrono::high_resolution_clock::now();

      std::chrono::duration<double, std::milli> fp_ms1 = t2 - t1;
        
      std::chrono::duration<double, std::milli> fp_ms2 = t3 - t2;

      std::cout << "(" <<  (uint)num_it <<  "," << (uint)vect_size << ") Standard loop: " << fp_ms1.count() << "ms, Parallel loop: " <<  fp_ms2.count()  << std::endl << std::endl;
      
      return (val1 ==  val2);
}
        
int main()
{
    uint rows = 480, cols = 640;
  
    cv::Mat img = cv::Mat::ones(rows, cols, CV_32FC1) * 1.5;
    
    for (uint i = 1; i < 30; ++i)
    {
      for(uint vect_size = 1; vect_size <9; ++vect_size)
      {
	bool ret = run_comparison(img,  i, vect_size);
      }

    }



}

#include <fstream>
#include <memory>
#include <chrono>

#include <omp.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

//#include <pips/utils/image_comparison_implementations.h>

      template<typename T>
      inline
uint16_t inner_loop(const cv::Mat& image, const T depth)
{

	int nRows = image.rows;
	int nCols = image.cols;
	      
	for( int i = 0; i < nRows; ++i)
	{
	  
	    const T* p = image.ptr<T>(i);
	    for(int j=0; j < nCols; ++j)
	    {
		T val = (p[j] < depth) ? 1 : 0;
		if(val)
		  return 1;
	    }

	}

	return 0;
}


/*
 * 200ms
  Vectorization factor: 8
  Vector inside of loop cost: 2
  Vector prologue cost: 17
  Vector epilogue cost: 18
  Scalar iteration cost: 3
  Scalar outside cost: 7
  Vector outside cost: 35
  prologue iterations: 4
  epilogue iterations: 4
  Calculated minimum iters for profitability: 10
  */
template<typename T>
      inline
/* Vectorizeable by gcc when using -ffast-math */
uint16_t vect2(const cv::Mat& image, const T depth)
{
  int nRows = image.rows;
  int nCols = image.cols;
	  
  if (image.isContinuous())
  {
      nCols *= nRows;
      nRows = 1;
  }
  
  int sum = 0; //doesn't matter if this is int or float

  for( int i = 0; i < nRows; ++i)
  {
  
    const T* p = image.ptr<T>(i);

    float intsum=0;

    for(int j=0; j <nCols; ++j)
    {

      
	//temp[j] = (p[j] < depth) ? 1 : 0;
	
	float t = (p[j] < depth) ? 1 : 0;
	//if( p[j] < depth)
	//  t = 1;
	
	
	
	intsum += t;// temp[j];
      
      

    }
      sum += intsum;

    
  }
  
  return sum;

}  


template<typename T>
      inline
/* Vectorizeable by gcc when using -ffast-math */
uint16_t vect7(const cv::Mat& image, const T depth)
{
  int nRows = image.rows;
  int nCols = image.cols;
	  
  if (image.isContinuous())
  {
      nCols *= nRows;
      nRows = 1;
  }
  
  uint main_loops = nCols / 8;
  uint remainder = nCols % 8;
  
  
  int sum = 0;

  for( int i = 0; i < nRows; ++i)
  {
  
    const T* p = image.ptr<T>(i);

    int a = 0;

    for(int j=0; j <main_loops; ++j)
    {
      float temp[8];
      float intsum=0;
      
      for(int k = 0; k < 8; ++k)
      {
	float t = (p[a] < depth) ? 1 : 0;
	intsum+=t;
	++a;
      
	/*
	temp[0] = (p[a + 0] < depth) ? 1 : 0;
	temp[1] = (p[a + 1] < depth) ? 1 : 0;
	temp[2] = (p[a + 2] < depth) ? 1 : 0;
	temp[3] = (p[a + 3] < depth) ? 1 : 0;
	temp[4] = (p[a + 4] < depth) ? 1 : 0;
	temp[5] = (p[a + 5] < depth) ? 1 : 0;
	temp[6] = (p[a + 6] < depth) ? 1 : 0;
	temp[7] = (p[a + 7] < depth) ? 1 : 0;
	
	
	intsum = temp[0] + temp[1] + temp[2] + temp[3] + temp[4] + temp[5] + temp[6] + temp[7];
	a+=8;
	*/
      
      }
      
      sum += intsum;
      
  // std::cout << "intsum: " << intsum<< "\t";


    }
    
    for(int k = 0; k < remainder; ++k)
    {
      uint8_t temp = (p[a] < depth) ? 1 : 0;
      a++;
      sum += temp;
    }

    
  }
  
  return sum;

}  




/*
  Vectorization factor: 4
  Vector inside of loop cost: 3
  Vector prologue cost: 13
  Vector epilogue cost: 17
  Scalar iteration cost: 4
  Scalar outside cost: 7
  Vector outside cost: 30
  prologue iterations: 2
  epilogue iterations: 2
  Calculated minimum iters for profitability: 7
  */
template<typename T>
      inline
/* Vectorizeable by gcc when using -ffast-math */
uint16_t vect6(const cv::Mat& image, const T depth)
{
  int nRows = image.rows;
  int nCols = image.cols;
	  
  if (image.isContinuous())
  {
      nCols *= nRows;
      nRows = 1;
  }
  
  int sum = 0;

  for( int i = 0; i < nRows; ++i)
  {
  
    const T* p = image.ptr<T>(i);

    int intsum=0;
    int temp[nCols];

    for(int j=0; j <nCols; ++j)
    {

      
	//temp[j] = (p[j] < depth) ? 1 : 0;
	
	int t = 0;
	if( p[j] < depth)
	  t = 1;
	
	
	
	intsum += t;// temp[j];
      
      

    }
      sum += intsum;

    
  }
  
  return sum;

}  

/*
 * 200ms
  Vectorization factor: 8
  Vector inside of loop cost: 2
  Vector prologue cost: 17
  Vector epilogue cost: 18
  Scalar iteration cost: 3
  Scalar outside cost: 7
  Vector outside cost: 35
  prologue iterations: 4
  epilogue iterations: 4
  Calculated minimum iters for profitability: 10
  */
template<typename T>
      inline
/* Vectorizeable by gcc when using -ffast-math */
uint16_t vect5(const cv::Mat& image, const T depth)
{
  int nRows = image.rows;
  int nCols = image.cols;
	  
  // This has slight benefits for uint16 when no obstacles (790 vs 870), but significant penalties when any obstacles exist
 /* if (image.isContinuous())
  {
      nCols *= nRows;
      nRows = 1;
  }
  */
  
  float sum = 0; //doesn't matter if this is int or float

  for( int i = 0; i < nRows; ++i)
  {
  
    const T* p = image.ptr<T>(i);

    T intsum=0;

    for(int j=0; j <nCols; ++j)
    {
	T t = (p[j] < depth) ? 1 : 0;
	intsum += t;
    }
    
    if(intsum)
    {
      return 1;
    }

    
  }
  
  return 0;

}  




/*
 * 288ms
  Vector inside of loop cost: 4
  Vector prologue cost: 21
  Vector epilogue cost: 27
  Scalar iteration cost: 4
  Scalar outside cost: 7
  Vector outside cost: 48
  prologue iterations: 4
  epilogue iterations: 4
  Calculated minimum iters for profitability: 11
  */

template<typename T>
      inline
/* Vectorizeable by gcc when using -ffast-math */
uint16_t vect4(const cv::Mat& image, const T depth)
{
  int nRows = image.rows;
  int nCols = image.cols;
	  
  if (image.isContinuous())
  {
      nCols *= nRows;
      nRows = 1;
  }
  
  uint16_t sum = 0;

  for( int i = 0; i < nRows; ++i)
  {
  
    const T* p = image.ptr<T>(i);

    uint16_t intsum=0;
    char temp[nCols];

    for(int j=0; j <nCols; ++j)
    {

      
	//temp[j] = (p[j] < depth) ? 1 : 0;
	
	char t = 0;
	if( p[j] < depth)
	  t = 1;
	
	
	
	intsum += t;// temp[j];
      
      

    }
      sum += intsum;

    
  }
  
  return sum;

}  



/*  
  Vector inside of loop cost: 16
  Vector prologue cost: 7
  Vector epilogue cost: 39
  Scalar iteration cost: 8
  Scalar outside cost: 6
  Vector outside cost: 46
  prologue iterations: 0
  epilogue iterations: 4
  Calculated minimum iters for profitability: 6
  */
template<typename T, uint V>
      inline
/* Vectorizeable by gcc when using -ffast-math */
uint16_t vect3(const cv::Mat& image, const T depth)
{
  int nRows = image.rows;
  int nCols = image.cols;
	  
  if (image.isContinuous())
  {
      nCols *= nRows;
      nRows = 1;
  }
  
  uint main_loops = nCols / V;
  uint remainder = nCols % V;
  
  
  uint16_t sum = 0;

  for( int i = 0; i < nRows; ++i)
  {
  
    const T* p = image.ptr<T>(i);

    int a = 0;

    for(int j=0; j <main_loops; ++j)
    {
      uint8_t temp[V];
      uint16_t intsum=0;
      

      for(int k=0; k < V; ++k)
      {
	temp[k] = (p[a] < depth) ? 1 : 0;
	intsum += temp[k];
	++a;
      }
      
      sum += intsum;
      
  // std::cout << "intsum: " << intsum<< "\t";


    }
    
    for(int k = 0; k < remainder; ++k)
    {
      int a = main_loops*V + k; 
      uint8_t temp = (p[a] < depth) ? 1 : 0;
      sum += temp;
    }

    
  }
  
  return sum;

}  


template<typename T>
uint16_t stock(const cv::Mat& img, const T depth)
{
 cv::Mat res;
      cv::compare(img, depth, res, cv::CMP_LT);

      int num_collisions = cv::countNonZero(res); 
      return num_collisions > 0;
}

      template<typename T, uint V, uint A>
      inline
size_t middle_loop(const cv::Mat& img, const T depth)
{
  size_t sum = 0;
  
  const uint16_t num_rows = img.rows;
  const uint16_t num_reduced_rows = img.rows / 4;
  const uint16_t num_cols = img.cols;
  const uint16_t num_reduced_cols = img.cols/4;

  auto t1 = std::chrono::high_resolution_clock::now();

  for(int i = 0; i < 100; ++i)
  {
    uint16_t x,y,width,height;
    
    x = i % num_reduced_cols;
    
    y = i % num_reduced_rows;
    width = num_cols - 2*x;
    
    height = num_rows - y;
    
    cv::Rect rect(x,y,width,height);
    cv::Mat roi(img,rect);
    
    roi = img;
    
    //Note: I should really also use 'roi' if I want to test how well does with nonaligned accesses
    uint16_t val;
    switch(A)
    {
      case 0: 
	val = inner_loop<T>(roi, depth);
	break;
      case 1:
	val = stock(roi,depth);
	break;
      case 2:
	val = vect2<T>(roi, depth);
	break;
      case 3:
	val = vect3<T, V>(roi, depth);
	break;
      case 4:
	val = vect4<T>(roi, depth);
	break;
      case 5:
	val = vect5<T>(roi, depth);
	break;
      case 7:
	val = vect7<T>(roi, depth);
	break;
    }
	
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


      template<uint V, uint A>
      inline
size_t parallel_outer_loop(const cv::Mat& img, uint8_t num_it, bool parallelism_enabled)
{
 
  std::vector<size_t> results(num_it); 
  

    
    #pragma omp parallel for schedule(dynamic) if(parallelism_enabled) //schedule(dynamic)
    for(uint8_t i = 0; i < num_it; i++)
    {
      float depth = (float)i / num_it;      
      //This condition increases time from 170 to 210 for vect5
      
      size_t result;
      if(img.type() == CV_32FC1)
      {
	result = middle_loop<float,V,A>(img, depth);
      }
      else if (img.type() == CV_16UC1)
      {
	float scaled_depth = std::numeric_limits<uint16_t>::max()*depth;
	result = middle_loop<uint16_t,V,A>(img, scaled_depth);
      }
      
	
      results[i] = result;
    }
    
    size_t sum = 0;
    for(uint8_t i = 0; i < num_it; i++)
    {
	sum+= results[i];
    }
  
  return sum;
}

      template<uint V>
      inline
bool run_comparison(const cv::Mat& img, uint8_t num_it)
{
      auto t1 = std::chrono::high_resolution_clock::now();

      size_t val1 = parallel_outer_loop<V,1>(img,  num_it, false);
  
      auto t2 = std::chrono::high_resolution_clock::now();
      
      size_t val2 = parallel_outer_loop<V,0>(img,  num_it, false);

      auto t3 = std::chrono::high_resolution_clock::now();
      
      size_t val3 = parallel_outer_loop<V,5>(img, num_it, false);
      
      auto t4 = std::chrono::high_resolution_clock::now();


      std::chrono::duration<double, std::milli> fp_ms1 = t2 - t1;
        
      std::chrono::duration<double, std::milli> fp_ms2 = t3 - t2;

      std::chrono::duration<double, std::milli> fp_ms3 = t4 - t3;

      std::cout << "(" <<  (uint)num_it <<  "," << (uint)V << ") stock: " << fp_ms1.count() << "ms, simple loop: " <<  fp_ms2.count()  << "ms, vect5: " << fp_ms3.count() << "ms" << std::endl << std::endl;
      
      std::cout << "values: " << val1 << ", " << val2 << ", " << val3 << std::endl;
      
      if(val3 !=  val2 || val3 != val1)
      {
	std::cout << "Error!" << std::endl;
      }
      
      return (val3 ==  val1);
}

void fill_mat(cv::Mat& img, float ratio)
{
  size_t num_pixels = img.rows * img.cols;
  
  int ind = 0;
      if(img.type() == CV_32FC1)
      {
	cv::MatIterator_<float> it, end;
        for( it = img.begin<float>(), end = img.end<float>(); it != end; ++it, ++ind)
	{
	  *it = 1 - ((float)ind * ratio / num_pixels);
	}
      }
      else if (img.type() == CV_16UC1)
      {
	cv::MatIterator_<uint16_t> it, end;
        for( it = img.begin<uint16_t>(), end = img.end<uint16_t>(); it != end; ++it, ++ind)
	{
	  *it = (uint16_t)(std::numeric_limits<uint16_t>::max() - (std::numeric_limits<uint16_t>::max() * ratio * ind / num_pixels));
	}
      }
}
        
int main()
{
    uint rows = 480, cols = 640;
  
    cv::Mat img1 = cv::Mat::zeros(rows, cols, CV_32FC1);
    cv::Mat img2 = cv::Mat::zeros(rows, cols, CV_16UC1);
    
    fill_mat(img1, .03);
    fill_mat(img2, .03);
    
    //std::cout << img1 << std::endl;
    
    //for (uint i = 1; i < 30; ++i)
    uint i = 3000;
    {
      std::cout << "32F:" << std::endl;
      bool ret = run_comparison<1>(img1,  i);
      /*ret = run_comparison<2>(img,  i);
      ret = run_comparison<4>(img,  i);
      ret = run_comparison<8>(img,  i);
      ret = run_comparison<16>(img,  i);
      */
      
      std::cout << "Uint16:" << std::endl;
      ret = run_comparison<1>(img2,  i);
      
    }



}
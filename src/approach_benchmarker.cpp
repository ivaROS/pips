#include <opencv2/core/core.hpp>
#include <opencv2/core/ocl.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <chrono>

struct datapoint
{
  double depth;
  std::chrono::time_point<std::chrono::high_resolution_clock> time;
  uint collisions;  
 
  datapoint(double depth, std::chrono::time_point<std::chrono::high_resolution_clock> time, uint collisions) :
    depth(depth), time(time), collisions(collisions) {}
};

  int sequentialEval(cv::UMat roi_cl, float depth)
  {
    cv::UMat res_cl;
    cv::compare(depth, roi_cl, res_cl, cv::CMP_GT);
    int num_collisions = cv::countNonZero(res_cl);
    return num_collisions;
  }


  cv::UMat getUMatROI(cv::UMat image_cl_, cv::Rect co_rect)
  {
   
      cv::UMat roi_cl(image_cl_,co_rect);
      return roi_cl;
  }

  
  
  
int run1(cv::Mat img)
{
  
  auto t1 = std::chrono::high_resolution_clock::now();  
  
  cv::UMat uimg = img.getUMat(cv::ACCESS_READ);
  
  
  cv::Rect roi(0,0,img.cols,img.rows);
  
  cv::UMat roiIm = getUMatROI(uimg,roi);
  
  std::vector<datapoint> res;

  
  auto t2 = std::chrono::high_resolution_clock::now();  
  

  for(float depth = .1; depth < 2.0; depth+=.01)
  {
    uint num_collisions = sequentialEval(roiIm, depth);
    auto t =  std::chrono::high_resolution_clock::now();  

    datapoint d(depth,t, num_collisions);
    res.push_back(d);
    
    //std::cout << "Depth = " << depth << ", collisions = " << num_collisions << std::endl;
    
  }
  
  std::cout << "Setup took " << std::chrono::duration<double, std::milli>(t2-t1).count() <<  "ms" << std::endl;

  std::chrono::time_point<std::chrono::high_resolution_clock> ptime = t2;
  
  for(auto d : res)
  {
    std::cout << "Depth = " << d.depth << ", collisions = " << d.collisions << ", in " << std::chrono::duration<double, std::milli>(d.time - ptime).count() << "ms" << std::endl;
    ptime = d.time;
  }
  
}

int main()
{
  cv::ocl::setUseOpenCL(true);
  int rows = 480, cols = 640;
  cv::Mat img = cv::Mat::ones(rows, cols, CV_32FC1);
  img *= 1.5;
  
  run1(img);
  
  cv::Mat img2 = cv::Mat::ones(rows, cols, CV_32FC1);
  img2 *= .8;
  run1(img2);
  
  
  cv::Mat img3 = cv::Mat::ones(rows-1, cols, CV_32FC1);
  img3 *= 1.5;
  run1(img3);
  
  cv::Mat img4 = cv::Mat::ones(rows, cols-1, CV_32FC1);
  img4 *= 1.5;
  run1(img4);
  
  cv::Mat img5 = cv::Mat::ones(rows, cols, CV_32FC1);
  img5 *= .9;
  run1(img5);
  
}
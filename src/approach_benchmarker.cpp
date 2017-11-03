#include <opencv2/core/core.hpp>
#include <opencv2/core/ocl.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <chrono>
#include <string>

struct datapoint
{
  double depth;
  std::chrono::duration<double, std::milli> duration;
  uint num_collisions;
 
  datapoint(double depth, std::chrono::time_point<std::chrono::high_resolution_clock> t1, std::chrono::time_point<std::chrono::high_resolution_clock> t2, uint num_collisions, uint num = 1) :
    depth(depth), duration(std::chrono::duration<double, std::milli>(t2-t1)/num), num_collisions(num_collisions/num) {}
    
  void print()
  {
     std::cout << "Depth = " << depth << ", with avg " << num_collisions << ", in average " << duration.count() << "ms" << std::endl;
  }
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
  
  std::chrono::time_point<std::chrono::high_resolution_clock> ptime = t2;


  for(float depth = .1; depth < 2.0; depth+=.01)
  {
    uint num_collisions = sequentialEval(roiIm, depth);
    auto t =  std::chrono::high_resolution_clock::now();  

    datapoint d(depth,ptime,t, num_collisions);
    ptime = t;
    res.push_back(d);    
  }
  
  std::cout << "Setup took " << std::chrono::duration<double, std::milli>(t2-t1).count() <<  "ms" << std::endl;
  
  std::cout << "First: ";
  res.front().print();
  std::cout << "Last: ";
  res.back().print();
  /*
  for(auto d : res)
  {
    std::cout << "Depth = " << d.depth << ", collisions = " << d.collisions << ", in " << d.duration.count() << "ms" << std::endl;
  }
  */
  return 1;
}



int runbatch(cv::Mat img, std::vector<float> depths, uint batch_size, int op, bool printout = true)
{
  
  auto t1 = std::chrono::high_resolution_clock::now();  
  
  cv::UMat uimg = img.getUMat(cv::ACCESS_READ);
  
  
  cv::Rect roi(0,0,img.cols,img.rows);
  
  cv::UMat roiIm = getUMatROI(uimg,roi);
  
  std::vector<datapoint> res;

  
  auto t2 = std::chrono::high_resolution_clock::now();  
  

  for(uint i = 0; i < depths.size(); i+=batch_size)
  {
    auto ts =  std::chrono::high_resolution_clock::now();  

    cv::UMat res_cl[batch_size];

    for(uint j = 0; j < batch_size; ++j)
    {
      float depth = depths[i + j];
      cv::compare(depth, roiIm, res_cl[j], op);
    }
    
    uint num_collisions[batch_size];
    for(uint j = 0; j < batch_size; ++j)
    {
      num_collisions[j] = cv::countNonZero(res_cl[j]);
    }
    
    uint total_collisions = 0;
    for(uint j = 0; j < batch_size; ++j)
    {
      total_collisions += num_collisions[j];
    }

    auto t =  std::chrono::high_resolution_clock::now();  
    
    

    datapoint d(0,ts,t, total_collisions, batch_size);

    res.push_back(d);    

  }
  
  if(printout)
  {
    std::cout << "Image size = " << img.cols << "x" << img.rows << std::endl;

    std::cout << "Setup took " << std::chrono::duration<double, std::milli>(t2-t1).count() <<  "ms" << std::endl;
    
    std::cout << "First: ";
    res.front().print();
    std::cout << "Last: ";
    res.back().print();
  }
  /*
  for(auto d : res)
  {
    std::cout << "Depth = " << d.depth << ", collisions = " << d.collisions << ", in " << d.duration.count() << "ms" << std::endl;
  }
  */
  
  return 1;
}



/* opencl countNonZero implementation: https://github.com/opencv/opencv/blob/0f0f5652fb531d5ff4e45079662198dffa51e9d2/modules/core/src/stat.cpp#L1250
			kernel: https://github.com/opencv/opencv/blob/05b15943d6a42c99e5f921b7dbaa8323f3c042c6/modules/core/src/opencl/reduce.cl#L563
			  Note: does not block
			
   opencl compare implementation: https://github.com/opencv/opencv/blob/master/modules/core/src/arithm.cpp#L1133
			kernel: https://github.com/opencv/opencv/blob/05b15943d6a42c99e5f921b7dbaa8323f3c042c6/modules/core/src/opencl/arithm.cl#L432
			  Note: blocks!
*/

int main()
{
  cv::ocl::setUseOpenCL(true);
  
  std::vector<float> depths;
  for(float depth = .1; depth < 3; depth+=.01)
  {
    depths.push_back(depth);
  }
  
  
  int rows = 480, cols = 640;
  
  
  cv::Mat img6 = cv::Mat::ones(16,32, CV_32FC1);
  img6 *= 1.5;
  std::cout << "32x16" << std::endl;
  run1(img6);
  
  
  cv::Mat img = cv::Mat::ones(rows, cols, CV_32FC1);
  img *= 1.5;
  std::cout << "640x480" << std::endl;
  run1(img);
  
  std::cout << "640x480" << std::endl;
  cv::Mat img2 = cv::Mat::ones(rows, cols, CV_32FC1);
  img2 *= .8;
  run1(img2);
  
  
  std::cout << "640x479" << std::endl;
  cv::Mat img3 = cv::Mat::ones(rows-1, cols, CV_32FC1);
  img3 *= 1.5;
  run1(img3);
  
  std::cout << "639x480" << std::endl;
  cv::Mat img4 = cv::Mat::ones(rows, cols-1, CV_32FC1);
  img4 *= 1.5;
  run1(img4);
  
  std::cout << "640x480" << std::endl;
  cv::Mat img5 = cv::Mat::ones(rows, cols, CV_32FC1);
  img5 *= .9;
  run1(img5);
  
  
  std::vector<int> oplist = {cv::CMP_GE, cv::CMP_GT, cv::CMP_LE, cv::CMP_LT};
  std::vector<std::string> opnames = {">=",">","<=","<"};
  
  std::vector<cv::Mat> imgs = {img6, img, img2, img3, img4, img5};
  
  for(uint batch_size = 1; batch_size < 5; ++batch_size)
  {
    std::cout << std::endl << "Batch size = " << batch_size  << std::endl;
    for(uint opnum = 0; opnum < oplist.size(); ++opnum)
    {
      int op = oplist[opnum];
      std::cout << "Operation: " << opnames[opnum] << std::endl;
      for(cv::Mat im : imgs)
      {
	auto t1 = std::chrono::high_resolution_clock::now();  
	runbatch(im,depths,batch_size,op,false);
	auto t2 = std::chrono::high_resolution_clock::now();  
	std::cout << "Image size = " << im.cols << "x" << im.rows << ": " << std::chrono::duration<double, std::milli>(t2-t1).count() <<  "ms" << std::endl;

      }
    }
  }

  
}
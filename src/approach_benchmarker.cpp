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


template<typename T>
int runbatch(cv::UMat img, std::vector<T> depths, uint batch_size, int op, bool printout = true)
{
  
  auto t1 = std::chrono::high_resolution_clock::now();  
  
  std::vector<datapoint> res;

  
  auto t2 = std::chrono::high_resolution_clock::now();  
  

  for(uint i = 0; i < depths.size(); i+=batch_size)
  {
    auto ts =  std::chrono::high_resolution_clock::now();  

    cv::UMat res_cl[batch_size];

    for(uint j = 0; j < batch_size && (i + j) < depths.size(); ++j)
    {
      T depth = depths[i + j];
      cv::compare(depth, img, res_cl[j], op);
    }
    
    uint num_collisions[batch_size];
    for(uint j = 0; j < batch_size && (i + j) < depths.size(); ++j)
    {
      num_collisions[j] = cv::countNonZero(res_cl[j]);
    }
    
    uint total_collisions = 0;
    for(uint j = 0; j < batch_size && (i + j) < depths.size(); ++j)
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
  
  int rows = 480, cols = 640;
  
  
  cv::Mat img = cv::Mat::ones(rows, cols, CV_32FC1) * 1.5;

  
  std::vector<float> depths;
  std::vector<cv::UMat> depthMats;

  for(float depth = .1; depth < 3; depth+=.01)
  {
    depths.push_back(depth);
    
    cv::Mat cmpimg = cv::Mat::ones(rows, cols, CV_32FC1) * depth;
    cv::UMat cmpuimg = cmpimg.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    depthMats.push_back(cmpuimg);
    
  }
  
  

  
  cv::UMat uimg = img.getUMat(cv::ACCESS_READ);


  
  cv::Rect roi(0,0,img.cols,img.rows);
  cv::Rect roi1(0,0,img.cols/2,img.rows);
  cv::Rect roi2(0,0,img.cols,img.rows/2);
  cv::Rect roi3(img.cols/2,0,img.cols/2,img.rows);
  cv::Rect roi4(0,img.rows/2,img.cols,img.rows/2);

  std::vector<cv::Rect> rois = {roi, roi1, roi2, roi3, roi4};

  
  std::vector<int> oplist = {cv::CMP_GE, cv::CMP_GT, cv::CMP_LE, cv::CMP_LT};
  std::vector<std::string> opnames = {">=",">","<=","<"};
    
  for(uint batch_size = 1; batch_size < 5; ++batch_size)
  {
    std::cout << std::endl << "Batch size = " << batch_size  << std::endl;
    for(uint opnum = 0; opnum < oplist.size(); ++opnum)
    {
      int op = oplist[opnum];
      std::cout << "Operation: " << opnames[opnum] << std::endl;
      for(cv::Rect rect : rois)
      {
	auto t1 = std::chrono::high_resolution_clock::now();  
	cv::UMat im = getUMatROI(uimg,rect);
	runbatch(im,depths,batch_size,op,false);
	auto t2 = std::chrono::high_resolution_clock::now();  
	std::cout << "Image size = " << im.cols << "x" << im.rows << ": " << std::chrono::duration<double, std::milli>(t2-t1).count() <<  "ms" << std::endl;

      }
    }
  }

  
  std::cout << "With mats:" << std::endl;
  
  for(uint batch_size = 1; batch_size < 5; ++batch_size)
  {
    std::cout << std::endl << "Batch size = " << batch_size  << std::endl;
    for(uint opnum = 0; opnum < oplist.size(); ++opnum)
    {
      int op = oplist[opnum];
      std::cout << "Operation: " << opnames[opnum] << std::endl;
      for(cv::Rect rect : rois)
      {
	std::vector<cv::UMat> roiDepthMats;
	for(cv::UMat depthmat : depthMats)
	{
	  cv::UMat roiDepthMat = getUMatROI(depthmat,rect);
	  roiDepthMats.push_back(roiDepthMat);
	}
	
	
	auto t1 = std::chrono::high_resolution_clock::now();  
	cv::UMat im = getUMatROI(uimg,rect);
	runbatch(im,roiDepthMats,batch_size,op,false);
	auto t2 = std::chrono::high_resolution_clock::now();  
	std::cout << "Image size = " << im.cols << "x" << im.rows << ": " << std::chrono::duration<double, std::milli>(t2-t1).count() <<  "ms" << std::endl;

      }
    }
  }
  
}
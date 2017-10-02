
#include "pips/collision_testing/robot_models/rectangular_model_pf.h"

//#include <sensor_msgs/Image.h>
//#include <geometry_msgs/TransformStamped.h>

//#include <opencv/cv.h>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"



//#include <Eigen/Eigen>
//#include <image_transport/image_transport.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <cv_bridge/cv_bridge.h>

//#include <iomanip>      // std::setprecision

RectangularModelPF::RectangularModelPF()
{
    name_ = "RectangularModelPF";
}


class ParallelLessThan : public cv::ParallelLoopBody
{
public:
  ParallelLessThan (const cv::Mat& img, cv::Mat& result, const float depth) :
    image(img),
    rowRes(result),
    depth(depth),
    nCols(image.cols)
  {

  }
  
  virtual void operator ()(const cv::Range& range) const
  {
//ROS_INFO_STREAM("Parallel for thread # = " << cv::getThreadNum());
    for (int r = range.start; r < range.end; r++)
    {
      const float* p = image.ptr<float>(r);
      bool stillGood = true;
      for ( int j = 0; stillGood && j < nCols; ++j)
      {
	float a = p[j];
          if(a < depth)
          {
            rowRes.at<uint8_t>(0,r)=1;
            stillGood = false;
	    //std::cout << "Collision detected in row " << r << ", column " << j << "by thread # " << cv::getThreadNum() << "\n";
          }
            
      }
      
    }
    
  }
  
private:
  const cv::Mat &image;
  cv::Mat &rowRes;
  float depth;
  int nCols;
  
};

bool RectangularModelPF::isLessThan(const cv::Mat& image, const float depth)
{
      //cv::Mat image = cv::Mat::zeros(4,10,img.type());
      //image += .3;

    int nRows = image.rows;
    

    cv::Mat rowRes  = cv::Mat::zeros(1,nRows,CV_8UC1);

    ParallelLessThan parallelLessThan(image, rowRes, depth);
    cv::parallel_for_(cv::Range(0, nRows), parallelLessThan);
    
    int num_nonzero = cv::countNonZero(rowRes);
    return num_nonzero > 0;

}

/*
bool RectangularModelPF::isLessThan(const cv::Mat& image, const float depth)
{
  int nRows = image.rows;
  int nCols = image.cols;
  
  cv::Mat rowRes  = cv::Mat(1,nCols,CV_8UC1);
  
  cv::parallel_for_(cv::Range(0, nRows), [&](const cv::Range& range){
    for (int r = range.start; r < range.end; r++)
    {
      const float* p = image.ptr<float>(r);
      bool stillGood = true;
      for ( int j = 0; stillGood && j < nCols; ++j)
      {
          if(p[j] < depth)
          {
            rowRes.at<uint8_t>(1,r)=1;
            stillGood = false;
          }
            
      }
      
    }
  });

  return cv::countNonZero(rowRes > 0);
}

*/



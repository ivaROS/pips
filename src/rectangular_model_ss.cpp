
#include "pips/collision_testing/robot_models/rectangular_model_ss.h"

#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>

//#include <opencv/cv.h>

#include <opencv2/core/core.hpp>
//#include <opencv2/core/ocl.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"  //If not using imshow, should remove this


#include <Eigen/Eigen>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <iomanip>      // std::setprecision


RectangularModelSS::RectangularModelSS()
{
    name_ = "RectangularModelSS";
}

bool RectangularModelSS::isLessThan(const cv::Mat& image, const float depth)
{

  int nRows = image.rows;
  int nCols = image.cols;

  
  //Could use templates to remove the duplication of this code
  if(image.depth() == CV_32FC1)
  {
    int i,j;
    const float* p;
    for( i = 0; i < nRows; ++i)
    {
        p = image.ptr<float>(i);
        for ( j = 0; j < nCols; ++j)
        {
            if(p[j] < depth)
            {
              return true;
            }
              
        }
    }
  }
  else if (image.depth() == CV_16UC1)
  {
    int i,j;
    const unsigned short int* p;
    for( i = 0; i < nRows; ++i)
    {
        p = image.ptr<unsigned short int>(i);
        for ( j = 0; j < nCols; ++j)
        {
            if(p[j] < depth)
            {
              return true;
            }
              
        }
    }
  }
  
  return false;
}


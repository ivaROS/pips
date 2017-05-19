
#include "pips/collision_testing/robot_models/rectangular_model_ocl.h"

#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>

//#include <opencv/cv.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/ocl.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"  //If not using imshow, should remove this


#include <Eigen/Eigen>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <iomanip>      // std::setprecision


RectangularModelOCL::RectangularModelOCL()
{
  //INFO ABOUT OpenCV BUILD
  std::cout << cv::getBuildInformation();
  // INFO ABOUT OpenCL
  std::cout << "OpenCL: " << std::endl;
  
  if(cv::ocl::haveOpenCL())
        {
            cv::ocl::setUseOpenCL(true);

            cv::ocl::Context mainContext;

            std::cout << "OPENCV_OPENCL_DEVICE=" << getenv("OPENCV_OPENCL_DEVICE") << std::endl;
            cv::ocl::Context& context = cv::ocl::Context::getDefault();
            for (size_t i = 0; i < context.ndevices(); i++) {
              std::cout << context.device(i).name() << std::endl;
            }

            
            if (!mainContext.create(cv::ocl::Device::TYPE_ALL))
            {
               std::cout << "Unable to create OpenCL Context" << std::endl;
            }
            

            for (unsigned int i = 0; i < mainContext.ndevices(); i++)
            {
                cv::ocl::Device device = mainContext.device(i);
                std::cout << "Device Name: " << device.name().c_str() << std::endl
                << "Available: "<< device.available() << std::endl
                << "imageSupport: " << device.imageSupport() << std::endl
                << "OpenCL_C_Version: " << device.OpenCL_C_Version().c_str() << std::endl;
            }



            //cv::ocl::Device(mainContext->device(0)); //Here is where you change which GPU to use (e.g. 0 or 1)
        }
  
  
  
  
  
  /*
  
  std::vector<ocl::PlatformInfo> platform_info;
  cv::ocl::getPlatfomsInfo(platform_info);
  for (size_t i = 0; i < platform_info.size(); i++)
  {
      std::cout
          << "\tName: " << platform_info[i].name() << endl
          << "\tVendor: " << platform_info[i].vendor() << endl
          << "\tVersion: " << platform_info[i].version() << endl
          << "\tDevice Number: " << platform_info[i].deviceNumber() << endl
          << std::endl;
  }
  */
  
    name_ = "RectangularModelOCL";
    
 //   std::vector<ocl::Info> param;
 // ocl::getDevice(param, ocl::CVCL_DEVICE_TYPE_GPU);
}


void RectangularModelOCL::doPrecomputation()
{
  image_cl_ = image_ref_.getUMat(cv::ACCESS_READ);
}

/* Takes in the position of robot base in camera coordinate frame */
bool RectangularModelOCL::testCollisionImpl(const cv::Point3d pt)
{
    cv::Rect co_rect = getCollisionRect(pt);
    
    //The collision object rectangle is our ROI in the original image
    cv::UMat roi_cl(image_cl_,co_rect);

    double depth = co_depth*scale_;
    cv::UMat res_cl;
    cv::compare(roi_cl, depth, res_cl, cv::CMP_LT);

    int num_collisions = cv::countNonZero(res_cl);
    bool collided = (num_collisions > 0);
    
    return collided;
}



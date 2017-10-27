
#include "pips/collision_testing/robot_models/rectangular_model_ss.h"

#include <pips/utils/image_comparison_implementations.h>


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

// Should be renamed 'SC' for 'Short Circuit'
RectangularModelSS::RectangularModelSS()
{
    name_ = "RectangularModelSS";
}

bool RectangularModelSS::isLessThan(const cv::Mat& image, const float depth)
{


  return utils::isLessThan::vectorized(image, depth);
}


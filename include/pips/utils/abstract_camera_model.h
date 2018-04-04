#ifndef PIPS_UTILS_ABSTRACT_CAMERA_MODEL
#define PIPS_UTILS_ABSTRACT_CAMERA_MODEL


#include <opencv2/core/core.hpp>


namespace pips
{
  namespace utils
  {
    //template <typename M>
    class AbstractCameraModel
    {
      public:
        virtual cv::Point2d project3dToPixel(const cv::Point3d& point)=0;
        virtual cv::Point3d projectPixelTo3dRay(const cv::Point2d& point)=0;
        
        virtual std::vector<int> getColumnRange(int left, int right)=0;
        
        virtual void update()=0;
        
        virtual float getPixelValue(const cv::Point3d& point)=0;
    };
  
  }


}


#endif //PIPS_UTILS_ABSTRACT_CAMERA_MODEL
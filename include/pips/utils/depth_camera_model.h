#ifndef PIPS_UTILS_DEPTH_CAMERA_MODEL
#define PIPS_UTILS_DEPTH_CAMERA_MODEL


#include <pips/utils/abstract_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/assert.h>

//#include <opencv2/core/core.hpp>


namespace pips
{
  namespace utils
  {
    
      
    class DepthCameraModel : public AbstractCameraModel
    {
      private:
        image_geometry::PinholeCameraModel model_;
        sensor_msgs::CameraInfoConstPtr info_msg_;
        
      public:
        
        void setInfo(const sensor_msgs::CameraInfoConstPtr& info_msg)
        {
          info_msg_ = info_msg;
        }
        
        void update()
        {
          model_.fromCameraInfo(info_msg_);
        }
        
        cv::Point2d project3dToPixel(const cv::Point3d& point) const
        {
          return model_.project3dToPixel(point);
        }
        
        cv::Point3d projectPixelTo3dRay(const cv::Point2d& point) const
        {
          return model_.projectPixelTo3dRay(point);
        }
        
        std::vector<int> getColumnRange(int left, int right) const
        {
          ROS_ASSERT(left <= right);
          
          std::vector<int> cols;
          
          for(int i = left; i < right; ++i)
          {
            cols.push_back(i);
          }
          
          return cols;
        }
        
        float getPixelValue(const cv::Point3d& point) const
        {
          return point.z;
        }
      
      
    };
  
  }


}


#endif  // PIPS_UTILS_DEPTH_CAMERA_MODEL

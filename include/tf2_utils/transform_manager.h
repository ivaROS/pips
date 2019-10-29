#ifndef TF2_UTILS_TRANSFORM_MANAGER_H
#define TF2_UTILS_TRANSFORM_MANAGER_H

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>

namespace tf2_utils
{

  class TransformManager
  {
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;

    
    void initialize(ros::NodeHandle nh=ros::NodeHandle(), ros::Duration buffer_size=ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME))
    {
      buffer_ = std::make_shared<tf2_ros::Buffer>(buffer_size);
      listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, nh);
    }
    
public:
  

    TransformManager(ros::Duration buffer_size=ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME))
    {
      ros::NodeHandle nh;
      initialize(nh, buffer_size);
    }
    
    TransformManager(bool create)
    {
      if(create)
      {
        initialize();
      }
    }
    
    TransformManager(TransformManager tfm, ros::NodeHandle nh, ros::Duration buffer_size=ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME))
    {
      if(!tfm.buffer_)
      {
        initialize(nh, buffer_size);
      }
      else
      {
        buffer_ = tfm.buffer_;
        listener_ = tfm.listener_;
      }
    }
    
    TransformManager(std::shared_ptr<tf2_ros::Buffer> buffer, std::shared_ptr<tf2_ros::TransformListener> listener):
      buffer_(buffer),
      listener_(listener)
    {}
    
    std::shared_ptr<tf2_ros::Buffer> getBuffer()
    {
      return buffer_;
    }
    
  };

} //end namespace tf2_utils

#endif //TF2_UTILS_TRANSFORM_MANAGER_H

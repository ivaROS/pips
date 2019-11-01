#ifndef PIPS_PARAM_UTILS_H
#define PIPS_PARAM_UTILS_H

#include<ros/console.h>
#include<ros/node_handle.h>

namespace pips
{

namespace utils
{
  
  template<typename T, typename S>
  bool searchParam(const ros::NodeHandle& nh, const std::string& name, T& value, const S& default_val)
  {
    std::string key;
    if (nh.searchParam(name, key))
    {
      nh.getParam(key, value );
    }
    else
    {
      value=default_val;
      ROS_WARN_STREAM("Warning, no entry found on parameter server for '" << name << "'! Using default value: '" << default_val << "'");
    }
    return true;
  }
  
  template<typename T>
  bool searchParam(const ros::NodeHandle& nh, const std::string& name, T& value)
  {
    std::string key;
    if (nh.searchParam(name, key))
    {
      nh.getParam(key, value );
      return true;
    }
    else
    {
      ROS_ERROR_STREAM("Error, no entry found on parameter server for '" << name << "'! No default value provided!");
      return false;
    }
  }
    
}

}

#endif //PIPS_PARAM_UTILS_H

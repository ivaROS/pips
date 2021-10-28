#ifndef PIPS_PARAM_UTILS_H
#define PIPS_PARAM_UTILS_H

#include<ros/console.h>
#include<ros/node_handle.h>

namespace pips
{

namespace utils
{
  bool searchParamKey(const ros::NodeHandle& nh, const std::string& name, std::string& key, int num_attempts);
  
  template<typename T>
  bool searchParamImpl(const ros::NodeHandle& nh, const std::string& name, T& value, int num_attempts)
  {
    std::string key;
    if (searchParamKey(nh, name, key, num_attempts))
    {
      nh.getParam(key, value);
      return true;
    }
    return false;
  }
  
  template<typename T, typename S>
  bool searchParam(const ros::NodeHandle& nh, const std::string& name, T& value, const S& default_val, int num_attempts=1)
  {
    if (!searchParamImpl(nh, name, value, num_attempts))
    {
      value=default_val;
      ROS_WARN_STREAM("Warning, no entry found on parameter server for '" << name << "'! Using default value: '" << default_val << "'");
    }
    return true;
  }
  
  template<typename T>
  bool searchParam(const ros::NodeHandle& nh, const std::string& name, T& value, int num_attempts=100)
  {
    if (!searchParamImpl(nh, name, value, num_attempts))
    {
      ROS_ERROR_STREAM("Error, no entry found on parameter server for '" << name << "'! No default value provided!");
      return false;
    }
    return true;
  }
    
}

}

#endif //PIPS_PARAM_UTILS_H

#ifndef PIPS_PARAM_UTILS_H
#define PIPS_PARAM_UTILS_H

#include<ros/console.h>
#include<ros/node_handle.h>

namespace pips
{

namespace utils
{
  
  template <typename T>
  bool get_param(const ros::NodeHandle& nh, const std::string& name, T& value, int num_attempts=100)
  {
    ros::WallRate rate(50);
    for(int i=0; i < num_attempts; i++)
    {
      if(nh.getParam(name, value))
      {
        return true;
      }

      if(i < num_attempts - 1)
      {  
        ROS_DEBUG_STREAM_NAMED("parameter_search", "Either failed to contact master or '" << name << "' was not found on parameter server; retrying...");
        rate.sleep();    
      }
    }
    ROS_WARN_STREAM("Warning, no entry found on parameter server for '" << name << "'! No default value provided!");
    return false;
  }
  
  template <typename T>
  bool get_param(const ros::NodeHandle& nh, const std::string& name, T& value, const T& default_val, int num_attempts=10)
  {
    if(!get_param(nh, name, value, num_attempts))
    {
      value = default_val;
      nh.setParam(name, default_val);
      ROS_WARN_STREAM("Warning, no entry found on parameter server for '" << name << "'! Using default value: '" << default_val << "'");
    }
    return true;
  }

  bool searchParamKey(const ros::NodeHandle& nh, const std::string& name, std::string& key, int num_attempts);
  
  template<typename T>
  bool searchParamImpl(const ros::NodeHandle& nh, const std::string& name, T& value, int num_attempts)
  {
    std::string key;
    if (searchParamKey(nh, name, key, num_attempts))
    {
      return get_param(nh, key, value, num_attempts);
    }
    return false;
  }
  
  template<typename T, typename S>
  bool searchParam(const ros::NodeHandle& nh, const std::string& name, T& value, const S& default_val, int num_attempts=10)
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

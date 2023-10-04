#include<pips/utils/param_utils.h>
#include<ros/console.h>
#include<ros/node_handle.h>

namespace pips
{

namespace utils
{
  
  bool searchParamKey(const ros::NodeHandle& nh, const std::string& name, std::string& key, int num_attempts)
  {
    ros::WallRate rate(50);
    for(int i = 0; i < num_attempts || num_attempts < 0; i++)
    {
      if (nh.searchParam(name, key))
      {
        ROS_DEBUG_STREAM_NAMED("parameter_search", "Search located key '" << key << "' on parameter server matching name '" << name << "'");
        return true;
      }

      if(i < num_attempts - 1)
      {  
        ROS_WARN_STREAM_NAMED("parameter_search", "Either failed to contact master or search was unable to find any match for '" << name << "' on parameter server; retrying...");
        rate.sleep();    
      }
    }
    return false;
  }
  
}

}

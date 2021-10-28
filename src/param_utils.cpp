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
        return true;
      }

      ROS_WARN_STREAM_NAMED("parameter_search", "Either failed to contact master or '" << name << "' was not found on parameter server; retrying...");
      if(i < num_attempts - 1)
      {  
        rate.sleep();    
      }
    }
    return false;
  }
  
}

}

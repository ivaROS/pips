#ifndef DURATION_ACCUMULATOR_H
#define DURATION_ACCUMULATOR_H


#include <ros/ros.h> // TODO: only include minimum requirements
#include <iomanip> // setprecision
#include <sstream> // stringstream

namespace pips
{
  
  namespace utils
  {


struct DurationAccumulator
{
  
private:
  ros::WallDuration total_duration;

  size_t num_samples=0;
  std::string name;


public:
  DurationAccumulator(std::string name) : name(name)
  {
  }
  
  void addDuration(ros::WallTime startTime, ros::WallTime endTime)
  {
    ros::WallDuration duration = (endTime - startTime);
    addDuration(duration);
    ROS_DEBUG_STREAM_NAMED(name + ".current_duration", toString(duration.toSec()));
    ROS_DEBUG_STREAM_NAMED(name + ".average_duration", "Average duration of " << num_samples << " samples: " << averageDuration());
  }
  
  void addDuration(ros::WallDuration duration)
  {
    total_duration += duration;
    num_samples++;
  }
  
  std::string averageDuration()
  {
    if(num_samples > 0)
    {
      double sec = total_duration.toSec()/((double)num_samples);
      
      return toString(sec);
      
    }
    else
      return "N/A";
  }
  
  std::string toString(double sec)
  {      
      std::stringstream stream;
      
      if(sec < 1e-6)
      {
	stream << std::fixed << std::setprecision(2) << sec * 1e9 << "ns";
      }
      else if(sec < 1e-3)
      {
	stream << std::fixed << std::setprecision(2) << sec * 1e6 << "us";
      }
      else if(sec < 1)
      {
	stream << std::fixed << std::setprecision(2) << sec * 1e3<< "ms";
      }
      else
      {
	stream << std::fixed << std::setprecision(2) << sec<< "s";
      }
      
      return stream.str();
  }

};

  }

}


#endif /* DURATION_ACCUMULATOR_H */
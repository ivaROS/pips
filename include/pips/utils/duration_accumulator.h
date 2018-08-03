#ifndef DURATION_ACCUMULATOR_H
#define DURATION_ACCUMULATOR_H


#include <ros/ros.h> // TODO: only include minimum requirements
#include <iomanip> // setprecision
#include <sstream> // stringstream

namespace pips
{
  
  namespace utils
  {

    
    struct Duration
    {
      double sec;
      
      Duration(double seconds) : sec(seconds) {}
      
      Duration(ros::WallDuration duration) : sec(duration.toSec()) {}
      
      operator double() {return sec;}
      
      operator std::string()
      {
        std::stringstream stream;
        
        stream << sec << "s";
        
        return stream.str();
      }
        
      
      std::string prettyPrint()
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

struct DurationAccumulator
{
  
private:
  ros::WallDuration total_duration;
  ros::WallDuration last_duration;

  size_t num_samples=0;
  std::string logger, name;


public:
  DurationAccumulator()
  {
  }
  
  void addDuration(ros::WallTime startTime, ros::WallTime endTime)
  {
    last_duration = (endTime - startTime);
    addDuration(last_duration);
    //ROS_DEBUG_STREAM_NAMED(logger, "[" << name << "] Current duration: " << toString(duration.toSec()));
    //ROS_DEBUG_STREAM_NAMED(logger, "[" << name << "] Average duration of " << num_samples << " samples: " << averageDuration());
  }
  
  void addDuration(ros::WallDuration duration)
  {
    total_duration += duration;
    num_samples++;
  }
  
  Duration averageDuration()
  {
    if(num_samples > 0)
    {
      Duration sec = total_duration.toSec()/((double)num_samples);
      
      return sec;
      
    }
    else
      return Duration(0);
  }
  
  int64_t getLastDuration()
  {
    return last_duration.toNSec();
  }
  
  size_t getNumSamples()
  {
    return num_samples;
  }
  

};

  }

}


#endif /* DURATION_ACCUMULATOR_H */

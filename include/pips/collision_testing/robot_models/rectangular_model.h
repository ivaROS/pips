#ifndef RECTANGULAR_MODEL
#define RECTANGULAR_MODEL

#include <pips/collision_testing/robot_models/hallucinated_robot_model.h>
#include <pips/utils/image_comparison_result.h>
#include <ros/ros.h>


class RectangularModel : public HallucinatedRobotModelImpl<cv::Point3d>
{

  public:
    RectangularModel();
    
    virtual void setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im);
    virtual bool inFrame(const cv::Point3d& pt);


  protected:


    /* Takes in the position of robot base in camera coordinate frame */
    virtual ComparisonResult testCollisionImpl(const cv::Point3d pt, CCOptions options);
    
    virtual cv::Mat generateHallucinatedRobotImpl(const cv::Point3d pt);
    
    virtual void getRawCollisionRect(const cv::Point3d pt, cv::Rect& co_rect, float& depth);

    virtual void getCollisionRect(const cv::Point3d pt, cv::Rect& co_rect, float& co_depth);
    

    
    virtual ComparisonResult isLessThan(const cv::Mat& image, const float depth);
    
    virtual cv::Rect getROIImpl(const cv::Point3d pt);
    
    ComparisonResult isLessThanDetails(const cv::Mat& image, const float depth);
    
    template<typename T>
    inline
    ComparisonResult isLessThanDetails(const cv::Mat& image, const T depth)
    {
      int nRows = image.rows;
      int nCols = image.cols;
      
      int i;
      
      const T* p;
      for( i = 0; i < nRows; ++i)
      {
        p = image.ptr<T>(i);
	for(int j = 0; j < nCols; ++j)
	{
	  T pixel_depth = p[j];
	  if(pixel_depth < depth)
	  {
	    //ROS_INFO_STREAM("p: " << p[j] << ", depth: " << depth << ", i: " << i << ", j: " << j);
	    ;
	    //pnt.x = j;
	    //pnt.y = i;
	    return ComparisonResult(i,j, pixel_depth);
	  }
	}
      }
      return ComparisonResult(false);
    }
    
  protected:
    std::vector<cv::Point3d> co_offsets_;
    ros::Publisher pub_;
    ros::NodeHandle nh_;
    

};

#endif // RECTANGULAR_MODEL

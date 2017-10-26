#ifndef RECTANGULAR_MODEL
#define RECTANGULAR_MODEL

#include <pips/collision_testing/robot_models/hallucinated_robot_model.h>
#include <ros/ros.h>


class RectangularModel : public HallucinatedRobotModelImpl<cv::Point3d>
{

  public:
    RectangularModel();
    
    virtual void setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im);
    virtual bool inFrame(const cv::Point3d& pt);


  protected:


    /* Takes in the position of robot base in camera coordinate frame */
    virtual bool testCollisionImpl(const cv::Point3d pt);
    
    virtual cv::Mat generateHallucinatedRobotImpl(const cv::Point3d pt);
    
    virtual void getRawCollisionRect(const cv::Point3d pt, cv::Rect& co_rect, float& depth);

    virtual void getCollisionRect(const cv::Point3d pt, cv::Rect& co_rect, float& co_depth);
    

    
    virtual bool isLessThan(const cv::Mat& image, const float depth);
    
    virtual cv::Rect getROIImpl(const cv::Point3d pt);
    
    virtual bool isLessThan(const cv::Mat& image, const float depth, cv::Point& pnt);
    
    template<typename T>
    inline
    bool isLessThan(const cv::Mat& image, const float depth, cv::Point& pnt)
    {
      int nRows = image.rows;
      int nCols = image.cols;
      
      int i;
      
      const T* p;
      for( i = 0; i < nRows; ++i)
      {
	int j = 0;
        p = image.ptr<T>(i);
	while((p[j] < depth) && (j < nCols))
	{
	    ++j;
	}
        if( j < nCols)
	{
	  pnt.x = i;
	  pnt.y = j;
	  return true;
	}
      }
      return false;
    }
    
  protected:
    std::vector<cv::Point3d> co_offsets_;
    ros::Publisher pub_;
    ros::NodeHandle nh_;
    

};

#endif // RECTANGULAR_MODEL

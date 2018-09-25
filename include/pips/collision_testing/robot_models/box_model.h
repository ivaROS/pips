#ifndef BOXMODEL_MODEL_H
#define BOXMODEL_MODEL_H

#include <pips/collision_testing/robot_models/hallucinated_robot_model.h>
#include <pips/collision_testing/robot_models/column_type.h>
#include <pips/collision_testing/geometry_models/box.h>
#include <math.h>


class BoxModel : public HallucinatedRobotModelImpl<geometry_msgs::Pose>
{
  private:
    pips::collision_testing::geometry_models::Box box_;
    
  public:
  
    BoxModel();
    
  
    void setParameters(double width, double height, double length, double rare_distance,  double safety_expansion, double floor_tolerance, bool show_im);
    void setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im);
    
   // virtual bool inFrame(const cv::Point3d& pt);

  protected:
  
    virtual ComparisonResult testCollisionImpl(const geometry_msgs::Pose pose, CCOptions options);
    
    virtual ComparisonResult isLessThan(const cv::Mat& col, float depth);
    virtual ComparisonResult isLessThanDetails(const cv::Mat& col, float depth);

        
    std::vector<COLUMN_TYPE> getColumns(const geometry_msgs::Pose pose);
        
    virtual cv::Mat generateHallucinatedRobotImpl(const geometry_msgs::Pose pt);
    
    geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose);
    
    /*
    virtual cv::Rect getColumnRect(int x, int y, int width, int height);

    virtual cv::Rect getROIImpl(const cv::Point3d pt);
    */
    
    //TODO added by Mahetem
    double robot_length, distance_from_rear;
    //double robot_length_,robot_width_, robot_height_;
    //end of addition

};


#endif /*  BOXMODEL_MODEL_H */


#include "hallucinated_robot_model.h"

#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>

//#include <opencv/cv.h>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


#include <Eigen/Eigen>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <iomanip>      // std::setprecision


void RectangularModel::setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im)
{
    cv::Point3d topr(radius+safety_expansion,-height,radius+safety_expansion);
    cv::Point3d topl(-radius-safety_expansion,-height,radius+safety_expansion);
    cv::Point3d bottomr(radius+safety_expansion,-floor_tolerance,radius+safety_expansion);
    cv::Point3d bottoml(-radius-safety_expansion,-floor_tolerance,radius+safety_expansion);

    //Note: at this time, the order doesn't matter. 
    cv::Point3d offsets[] = {topr,topl,bottoml,bottomr};
    std::vector<cv::Point3d> co_offsets(offsets, offsets + sizeof(offsets) / sizeof(cv::Point3d) );
    
    co_offsets_ = co_offsets;
    show_im_ = show_im;
}

/* Takes in the position of robot base in camera coordinate frame */
bool RectangularModel::testCollision(const cv::Point3d pt)
{


    cv::Point2d co_uv[4];
    double co_depth;


    for (int it = 0; it <4; ++it) 
    {
      //Add specified offsets to robot's base position
      cv::Point3d addedpnt = pt + co_offsets_[it];
      
      //Store the depth of the collision object
      co_depth = addedpnt.z;
      
      //Project point to pixel coordinates
      cv::Point2d uv = cam_model_->project3dToPixel(addedpnt);
      //ROS_DEBUG("Coords: %f, %f", uv.x, uv.y);
      co_uv[it] = uv;

    }


    //Using the 4 points, construct a rectangle
    double minXVal, maxXVal, minYVal,maxYVal;
    minYVal = std::min(image_ref_.rows-1.0,std::max(0.0,std::min(co_uv[0].y,co_uv[1].y)));
    minXVal = std::min(image_ref_.cols-1.0,std::max(0.0, std::min(co_uv[1].x,co_uv[2].x)));
    maxYVal = std::max(0.0, std::min(image_ref_.rows-1.0, std::max(co_uv[2].y,co_uv[3].y)));
    maxXVal = std::max(0.0, std::min(image_ref_.cols-1.0, std::max(co_uv[3].x,co_uv[0].x)));

    cv::Point2d topL(minXVal, minYVal);
    cv::Point2d bottomR(maxXVal, maxYVal);

    cv::Rect co_rect(topL, bottomR);

    //The following should be a more elegant way to take the collision outline and crop it to fit in the image
    //However, I would have to know which point was which. Rather than force an arbitrary order, a single-time call could create 2 points, one for topl, and one for bottomr r, that would be used in place of co_offsets_; that would also only require projecting 2 points, so no need for the loop.
    //cv::Rect co_rect1 = cv::Rect(cv::Point2d(co_uv[0].x, co_uv.y), cv::Point2d(co_uv[, bottomR) &= cv::Rect(Point(0, 0), image_ref_->size());

    //ROS_DEBUG_STREAM_THROTTLED(2, "[collision_checker] co_rect(current): " << co_rect << ", proposed: " << co_rect1);

    //The collision object rectangle is our ROI in the original image
    cv::Mat roi(image_ref_,co_rect);


    bool collided = isLessThan(roi, co_depth*scale_);
    
    return collided;
}

bool RectangularModel::isLessThan(const cv::Mat& image, float depth)
{
/*
    double min_depth;
    cv::minMaxLoc(image, &min_depth, NULL, NULL, NULL);

    return min_depth < depth;
    */
    
    //Built in approach:
/*
    cv::Mat collisions = (roi <= co_depth*scale_);
    int num_collisions = cv::countNonZero(collisions);
    bool collided = (num_collisions>0);
  */
    
    int nRows = image.rows;
    int nCols = image.cols;

    
    //Could use templates to remove the duplication of this code
    if(image.depth() == CV_32FC1)
    {
      int i,j;
      const float* p;
      for( i = 0; i < nRows; ++i)
      {
          p = image.ptr<float>(i);
          for ( j = 0; j < nCols; ++j)
          {
              if(p[j] < depth)
              {
                return true;
              }
                
          }
      }
    }
    else if (image.depth() == CV_16UC1)
    {
      int i,j;
      const unsigned short int* p;
      for( i = 0; i < nRows; ++i)
      {
          p = image.ptr<unsigned short int>(i);
          for ( j = 0; j < nCols; ++j)
          {
              if(p[j] < depth)
              {
                return true;
              }
                
          }
      }
    }
    
    return false;
}


cv::Mat RectangularModel::generateHallucinatedRobot(const cv::Point3d pt)
{
    cv::Mat viz = cv::Mat::zeros(image_ref_.rows, image_ref_.cols, image_ref_.type());
    
    cv::Point2d co_uv[4];
    double co_depth;

    for (int it = 0; it <4; ++it) {
      //Add specified offsets to robot's base position
      cv::Point3d addedpnt = pt + co_offsets_[it];
      
      //Store the depth of the collision object
      co_depth = addedpnt.z;
      
      //Project point to pixel coordinates
      cv::Point2d uv = cam_model_->project3dToPixel(addedpnt);
      //ROS_DEBUG("Coords: %f, %f", uv.x, uv.y);
      co_uv[it] = uv;

    }
    

    //Using the 4 points, construct a rectangle
    double minXVal, maxXVal, minYVal,maxYVal;
    minYVal = std::min(image_ref_.rows-1.0,std::max(0.0,std::min(co_uv[0].y,co_uv[1].y)));
    minXVal = std::min(image_ref_.cols-1.0,std::max(0.0, std::min(co_uv[1].x,co_uv[2].x)));
    maxYVal = std::max(0.0, std::min(image_ref_.rows-1.0, std::max(co_uv[2].y,co_uv[3].y)));
    maxXVal = std::max(0.0, std::min(image_ref_.cols-1.0, std::max(co_uv[3].x,co_uv[0].x)));

    cv::Point2d topL(minXVal, minYVal);
    cv::Point2d bottomR(maxXVal, maxYVal);

    cv::Rect co_rect(topL, bottomR);

    cv::Mat roi(viz,co_rect);
    roi = co_depth*scale_;
    
    return viz;
}




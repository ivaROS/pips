
#include "pips/collision_testing/robot_models/rectangular_model.h"

//#include <sensor_msgs/Image.h>
//#include <geometry_msgs/TransformStamped.h>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


//#include <Eigen/Eigen>
//#include <image_transport/image_transport.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <cv_bridge/cv_bridge.h>

//#include <iomanip>      // std::setprecision

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pips/utils/image_comparison_implementations.h>

//#include <pcl/conversions.h>
//#include <pcl_ros/transforms.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


RectangularModel::RectangularModel() : HallucinatedRobotModelImpl<cv::Point3d>()
{
  name_ = "RectangularModel";
  pub_ = nh_.advertise<PointCloud>("collisions",100);
}


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

/* One option would be to return an object that implements some comparable interface, allowing it to be used as boolean 'collided' variable, but also containing extra information that can be accessed
 * by classes that choose to
 */
bool RectangularModel::testCollisionImpl(const cv::Point3d pt)
{
  float co_depth;
  cv::Rect co_rect;
  getCollisionRect(pt,co_rect,co_depth);

  //The collision object rectangle is our ROI in the original image
  cv::Mat roi(image_ref_, co_rect);

  float depth = co_depth*scale_;
  bool collided = isLessThan(roi, depth);
  


  if(collided)
  {        
      cv::Point collisionPnt;
      isLessThan(roi,depth,collisionPnt);

      cv::Point offset;
      cv::Size size;
      roi.locateROI(size, offset);
      
      collisionPnt += offset;
      cv::Point3d ray = cam_model_->projectPixelTo3dRay(collisionPnt);
      cv::Point3d worldPoint = ray * co_depth;
      
      //ROS_INFO_STREAM("Collision: pixel = " << collisionPnt << ", offset = " << offset << ", roi = " << co_rect << ", depth = " << depth << ", ray = " << ray << ", worldpoint = " << worldPoint );
      
      PointCloud::Ptr msg (new PointCloud);
      msg->header.stamp = ros::Time::now().toNSec()/1e3;	//https://answers.ros.org/question/172241/pcl-and-rostime/
      msg->header.frame_id = cam_model_->cameraInfo().header.frame_id;// "camera_depth_optical_frame";
      msg->height = msg->width = 1;
      msg->points.push_back (pcl::PointXYZ(worldPoint.x, worldPoint.y, worldPoint.z));
      
      pub_.publish(msg);
  }
  

  return collided;
}

cv::Rect RectangularModel::getROIImpl(const cv::Point3d pt)
{
  float co_depth;
  cv::Rect co_rect;
  getCollisionRect(pt,co_rect,co_depth);
  return co_rect;
}

void RectangularModel::getRawCollisionRect(const cv::Point3d pt, cv::Rect& co_rect, float& depth)
{
  cv::Point2d co_uv[4];

  float co_depth;

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
  
  depth = co_depth;


  //Using the 4 points, construct a rectangle
  double minXVal, maxXVal, minYVal,maxYVal;
  minYVal = std::min(co_uv[0].y,co_uv[1].y);
  minXVal = std::min(co_uv[1].x,co_uv[2].x);
  maxYVal = std::max(co_uv[2].y,co_uv[3].y);
  maxXVal = std::max(co_uv[3].x,co_uv[0].x);

  cv::Point2d topL(minXVal, minYVal);
  cv::Point2d bottomR(maxXVal, maxYVal);

  co_rect = cv::Rect(topL, bottomR);
}

//TODO: pass pt by const reference
void RectangularModel::getCollisionRect(const cv::Point3d pt, cv::Rect& co_rect, float& depth)
{
  getRawCollisionRect(pt, co_rect, depth);
  
  co_rect &= getImageRect();
    
    //The following should be a more elegant way to take the collision outline and crop it to fit in the image
    //However, I would have to know which point was which. Rather than force an arbitrary order, a single-time call could create 2 points, one for topl, and one for bottomr r, that would be used in place of co_offsets_; that would also only require projecting 2 points, so no need for the loop.
    //cv::Rect co_rect1 = cv::Rect(cv::Point2d(co_uv[0].x, co_uv.y), cv::Point2d(co_uv[, bottomR) &= cv::Rect(Point(0, 0), image_ref_->size());
}

bool RectangularModel::inFrame(const cv::Point3d& pt)
{
  cv::Rect co_rect;
  float depth;
  getRawCollisionRect(pt, co_rect, depth);
  
  return co_rect == (co_rect & getImageRect());

}

bool RectangularModel::isLessThan(const cv::Mat& image, const float depth)
{
  bool collided = utils::isLessThan::stock(image, depth);
  return collided;
}

bool RectangularModel::isLessThan(const cv::Mat& image, const float depth, cv::Point& pnt)
{
  if(image.depth() == CV_32FC1)
  {
    return isLessThan<float>(image, depth, pnt);
  }
  else if (image.depth() == CV_16UC1)
  {
    return isLessThan<uint16_t>(image, depth, pnt);
  }
  ROS_INFO_STREAM("Not good!");
  return false;
}

cv::Mat RectangularModel::generateHallucinatedRobotImpl(const cv::Point3d pt)
{
    cv::Mat viz = HallucinatedRobotModelImpl::generateHallucinatedRobotImpl(pt);
    
    
  float co_depth;
  cv::Rect co_rect;
  getCollisionRect(pt,co_rect,co_depth);


    cv::Mat roi(viz,co_rect);
    roi = co_depth*scale_;
    
    return viz;
}




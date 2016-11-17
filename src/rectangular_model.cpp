
#include "rectangular_model.h"

#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv/cv.h>
#include <Eigen/Eigen>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

  CylindricalModel::CylindricalModel(double radius, double height, double safety_expansion, double floor_tolerance)
  {
      robot_radius_ = radius + safety_expansion;
      robot_height_ = height;
      floor_tolerance_ = floor_tolerance;
      
  }
  
  // Question: is it better to use the camera model's methods for clarity, or to use pure Eigen matrices for speed?
  // Initially, will use the model, but will likely switch over in the future.
  bool CylindricalModel::testCollision(const cv::Point3d pt)
  {
    double h_squared = pt.x*pt.x + pt.z*pt.z;
    double h = std::sqrt(h_squared);
    
    double theta_c = std::atan2(pt.x,pt.z);
    double theta_d = std::asin(robot_radius_/h);
    
    cv::Point3d Xt_lb(h*std::sin(theta_c - theta_d), pt.y+floor_tolerance_, h*std::cos(theta_c - theta_d));
    cv::Point3d Xt_rb(h*std::sin(theta_c + theta_d), pt.y+floor_tolerance_, h*std::cos(theta_c + theta_d));

    cv::Point3d Xt_lt = Xt_lb + cv::Point3d(0,-robot_height_,0);
    cv::Point3d Xt_rt = Xt_rb + cv::Point3d(0,-robot_height_,0);

    cv::Point2d p_lb = cam_model_.project3dToPixel(Xt_lb);
    cv::Point2d p_rb = cam_model_.project3dToPixel(Xt_rb);
    cv::Point2d p_lt = cam_model_.project3dToPixel(Xt_lt);
    cv::Point2d p_rt = cam_model_.project3dToPixel(Xt_rt);

    unsigned int p_lb_x = std::min(std::max(0,(int)std::floor(p_lb.x)),image_ref_.cols-1);
    unsigned int p_lb_y = std::max(std::min(image_ref_.rows-1,(int)std::ceil(p_lb.y)),0);
    cv::Point2i  p_lb_ind(p_lb_x, p_lb_y);

    unsigned int p_lt_x = std::max(0,std::min((int)floor(p_lt.x),image_ref_.cols-1));
    unsigned int p_lt_y = std::max(0,std::min((int)floor(p_lt.y),image_ref_.rows-1));
    cv::Point2i  p_lt_ind(p_lt_x, p_lt_y);

    unsigned int p_rb_x = std::max(0,std::min((int)ceil(p_rb.x),image_ref_.cols-1));
    unsigned int p_rb_y = std::max(std::min(image_ref_.rows,(int)ceil(p_rb.y)),1);
    cv::Point2i  p_rb_ind(p_rb_x, p_rb_y);

    unsigned int p_rt_x = std::max(0,std::min((int)ceil(p_rt.x),image_ref_.cols-1));
    unsigned int p_rt_y = std::max(0,std::min((int)ceil(p_rt.y),image_ref_.rows-1));
    cv::Point2i  p_rt_ind(p_rt_x, p_rt_y);
    
    cv::Mat viz;
    if(show_im_)
    {
        viz = image_ref_.clone();
    }
    
    
    /*    
    simulated_image(p_lt_ind(2):p_lb_ind(2),p_lb_ind(1)) = Xt_lb(3);
    simulated_image(p_rt_ind(2):p_rb_ind(2),p_rb_ind(1)) = Xt_rb(3);
    */
    
    /*Note: I should write my own matrix-matrix/matrix-scalar method that returns boolean as soon as condition satisfied*/
    cv::Mat lCol = image_ref_.col(p_lt_ind.x).rowRange(p_lt_ind.y,p_lb_ind.y);
    if(cv::countNonZero(lCol < Xt_lb.z) > 0)
    {
        if(show_im_)
        {
            viz.col(p_lt_ind.x).rowRange(p_lt_ind.y,p_lb_ind.y) = Xt_lb.z;
        }
        else
        {
            return false;
        }
    }
    
    
    if(cv::countNonZero((cv::Mat)image_ref_.col(p_rt_ind.x).rowRange(p_rt_ind.y,p_rb_ind.y) < Xt_rb.z*scale_) > 0)
    {
        if(show_im_)
        {
            viz.col(p_rt_ind.x).rowRange(p_rt_ind.y,p_rb_ind.y) = Xt_rb.z*scale_;
        }
        else
        {
            return false;
        }
    }


for(unsigned int p_x = p_lb_ind.x+1; p_x < p_rb_ind.x-1; ++p_x)
{
    
    /*%reprojecting x pixel coordinate at center of y coordinates to ray
    L = K\[p_x;K(2,3);1];
    %%Equivalent to the below 
    %L = K\p_reproj;    %reprojecting pixel to ray
    %L(2) = 0;
    */
    
    cv::Point3d ray = cam_model_.projectPixelTo3dRay(cv::Point2d(p_x,pt.y));
    //ray.y = 0;
    
    
    //equations for intersection of ray and circle
    double a = ray.x*ray.x + ray.z*ray.z;
    double b = -2*(ray.x*pt.x + ray.z*pt.z);
    double c = h_squared - robot_radius_*robot_radius_;
    
    ROS_ASSERT_MSG(b*b - 4*a*c >=0, "Complex solution for ray-circle intersection!");

    /*
    Solve for parameter t that yields intersection
    Note that we only care about the more distant intersection (the + solution)
    */
    double t = (-b + std::sqrt(b*b-4*a*c))/(2*a);
    
    //Get world coordinates of intersection
    cv::Point3d X_h = ray*t;
    
    //for bottom:
    cv::Point3d X_hb = X_h + cv::Point3d(0,floor_tolerance_,0);
    
    //for top:
    cv::Point3d X_ht = X_h + cv::Point3d(0,robot_height_,0);
    
    //project back to pixels to get y coordinate
    cv::Point2d p_xhb =  cam_model_.project3dToPixel(X_hb);
    cv::Point2d p_xht =  cam_model_.project3dToPixel(X_ht);
    
    cv::Rect column((int)std::floor(p_xht.x),(int)std::floor(p_xht.y),1,(int)std::ceil(p_xhb.y)-(int)std::floor(p_xht.y)); //need to make sure that this is equivalent to what I actually want. One advantage of using a Rect is that matrix bounds are automatically handled- False! they are not handled, unless I find union with a rect of the size of the image. 
    
    //cv::Mat column = image_ref_.col(p_xhb.x).rowRange(std::max(1,(int)std::floor(p_xht.y)),std::min(image_ref_.rows-1,(int)std::ceil(p_xhb.y)));
    
    if(cv::countNonZero(cv::Mat(image_ref_,column) < X_h.z*scale_)>0)
    {
        if(show_im_)
        {
            cv::Mat(viz,column) = X_h.z*scale_;
        }
        else
        {
            return false;
        }
    }
    
   } 
   
   return true;
  }



  
RectangularModel::RectangularModel(double radius, double height, double safety_expansion, double floor_tolerance)
{
    cv::Point3d topr(radius+safety_expansion,-height,radius+safety_expansion);
    cv::Point3d topl(-radius-safety_expansion,-height,radius+safety_expansion);
    cv::Point3d bottomr(radius+safety_expansion,-floor_tolerance,radius+safety_expansion);
    cv::Point3d bottoml(-radius-safety_expansion,-floor_tolerance,radius+safety_expansion);

    //Note: at this time, the order doesn't matter. 
    cv::Point3d offsets[] = {topr,topl,bottoml,bottomr};
    std::vector<cv::Point3d> co_offsets(offsets, offsets + sizeof(offsets) / sizeof(cv::Point3d) );
    co_offsets_ = co_offsets;
}

/* Takes in the position of robot base in camera coordinate frame */
bool RectangularModel::testCollision(const cv::Point3d pt)
{


    cv::Point2d co_uv[4];
    double co_depth;

    for (int it = 0; it <4; ++it) {
      //Add specified offsets to robot's base position
      cv::Point3d addedpnt = pt + co_offsets_[it];
      
      //Store the depth of the collision object
      co_depth = addedpnt.z;
      
      //Project point to pixel coordinates
      cv::Point2d uv = cam_model_.project3dToPixel(addedpnt);
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
    
    //Check if any points in ROI are closer than collision object's depth
/*
    cv::Mat collisions = (roi <= co_depth*scale_);
    int num_collisions = cv::countNonZero(collisions);
    bool collided = (num_collisions>0);
  */

    double min_depth;

    cv::minMaxLoc(roi, &min_depth, NULL, NULL, NULL);

    bool collided = min_depth < co_depth*scale_;  
    return collided;
}




#ifndef PIPS_GEOMETRY_MODELS_CYLINDER_H
#define PIPS_GEOMETRY_MODELS_CYLINDER_H

//#include <sensor_msgs/Image.h>
//#include <geometry_msgs/TransformStamped.h>

//#include <opencv/cv.h>

#include <pips/collision_testing/geometry_models/geometry_models.h>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"


//#include <Eigen/Eigen>
//#include <image_transport/image_transport.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <cv_bridge/cv_bridge.h>

//#include <iomanip>      // std::setprecision

#include "ros/ros.h" //Needed for 'ROS_BREAK and ROS_ASSERT



namespace pips
{
  namespace collision_testing
  {
    namespace geometry_models
    {
      

class Cylinder : public GeometryModel
{
    double radius_=-1, height_=-1;
    
public:
    Cylinder()
    {
      type_ = "Cylinder";
    }
  
    Cylinder(double radius, double height):
      radius_(radius),
      height_(height)
    {
      type_ = "Cylinder";
      
      marker_.scale.x = radius_*2;
      marker_.scale.y = radius_*2;
      marker_.scale.z = height_;

      marker_.type = visualization_msgs::Marker::CYLINDER;
    }


  COLUMN_TYPE getColumn(const cv::Point2d top, const cv::Point2d bottom, const float depth, int img_width, int img_height)
  {
    //By forming a Rect in this way, doesn't matter which point is the top and which is the bottom.
    cv::Rect_<double> r(top,bottom);
    
    int x = r.tl().x + .00000000001;  // Add a tiny number to handle cases where the process of projecting to ray, finding intersection, and projecting back to the image plane introduces a tiny numerical error, apparently only with smaller values that are odd. The added value will never push the number to the next integer value but is much larger than any error seen so far
    int y = std::floor(r.tl().y);
    int width = 1;
    int height = std::ceil(r.br().y)-y + 1; //ROI's from rectangles are noninclusive on the right/bottom sides, so need to add 1 to include the bottom row
    
  //The only changes needed to use a transposed image are swapping the x and y as well as width and height
    cv::Rect column = cv::Rect(x,y,width,height);
    cv::Rect imageBounds(0,0,img_width, img_height);
    cv::Rect bounded = column & imageBounds;
    
    COLUMN_TYPE col;
    
    col.rect = bounded;
    col.depth = depth;
     
    //std::cout << std::setprecision(16) << "top = " << top << ", bottom = " << bottom << ", Col = " << r << "tl: " << r.tl() << " br: " << r.br() << ", rect=" << column << ", bounded= " << bounded << "\n"; // Debugging printout to troubleshoot incorrect x values for columns. The 'magic number' added above resolved the problem.
    
    return col;
  }
  
  std::vector<COLUMN_TYPE> getColumns(const geometry_msgs::Pose& pose, int img_width, int img_height)
  {
    ROS_ASSERT(radius_>0 && height_ > 0);
    const cv::Point3d pt(pose.position.x, pose.position.y, pose.position.z);
    
    return getColumns(pt, img_width, img_height);
  }

  std::vector<COLUMN_TYPE> getColumns(const cv::Point3d pt, int img_width, int img_height)
  {    
    ROS_DEBUG_STREAM_NAMED(name_, "Get columns for " << pt);
    std::vector<COLUMN_TYPE> cols;
    
    unsigned int left = 0;
    unsigned int right = img_width;
    
    double h_squared = pt.x*pt.x + pt.z*pt.z;
    double h = std::sqrt(h_squared);
    
    cv::Point3d Xc_l, Xc_r;
    
    // We only calculate the actual side borders of the robot if it is far enough away that they could be seen
    if(h > radius_)
    {
      double tangentDist = std::sqrt(h_squared - radius_*radius_);
      
      double theta_c = std::atan2(pt.x,pt.z);
      double theta_d = std::asin(radius_/h);
      
      Xc_l = cv::Point3d(tangentDist*std::sin(theta_c - theta_d), pt.y, tangentDist*std::cos(theta_c - theta_d));
      Xc_r = cv::Point3d(tangentDist*std::sin(theta_c + theta_d), pt.y, tangentDist*std::cos(theta_c + theta_d));
    }

    
    {
      cv::Point3d Xt_lb = Xc_l + cv::Point3d(0,height_/2,0);
      cv::Point3d Xt_rb = Xc_r + cv::Point3d(0,height_/2,0);

      cv::Point3d Xt_lt = Xc_l + cv::Point3d(0,-height_/2,0);
      cv::Point3d Xt_rt = Xc_r + cv::Point3d(0,-height_/2,0);

      cv::Point2d p_lb = cam_model_->project3dToPixel(Xt_lb);
      cv::Point2d p_rb = cam_model_->project3dToPixel(Xt_rb);
      cv::Point2d p_lt = cam_model_->project3dToPixel(Xt_lt);
      cv::Point2d p_rt = cam_model_->project3dToPixel(Xt_rt);
      
      //std::cout << "pt= " << pt << "\nh_squared= " << h_squared << "\nh= " << h << "\ntheta_c= " << theta_c << "\ntheta_d= " << theta_d << "\nXt_lb= " << Xt_lb << "\nXt_rb= " << Xt_rb << "\n";

      unsigned int p_lb_x = std::min(std::max(0,(int)std::floor(p_lb.x)),img_width-1);
      unsigned int p_lb_y = std::max(std::min(img_height-1,(int)std::ceil(p_lb.y)),0);
      cv::Point2i  p_lb_ind(p_lb_x, p_lb_y);

      unsigned int p_lt_x = std::max(0,std::min((int)floor(p_lt.x),img_width-1));
      unsigned int p_lt_y = std::max(0,std::min((int)floor(p_lt.y),img_height-1));
      cv::Point2i  p_lt_ind(p_lt_x, p_lt_y);

      unsigned int p_rb_x = std::max(0,std::min((int)ceil(p_rb.x),img_width-1));
      unsigned int p_rb_y = std::max(std::min(img_height,(int)ceil(p_rb.y)),1);
      cv::Point2i  p_rb_ind(p_rb_x, p_rb_y);

      unsigned int p_rt_x = std::max(0,std::min((int)ceil(p_rt.x),img_width-1));
      unsigned int p_rt_y = std::max(0,std::min((int)ceil(p_rt.y),img_height-1));
      cv::Point2i  p_rt_ind(p_rt_x, p_rt_y);
      
      ROS_DEBUG_STREAM("pt= " << pt << "\nh_squared= " << h_squared << "\nh= " << h << /*"\ntheta_c= " << theta_c << "\ntheta_d= " << theta_d << */ "\nXc_l= " << Xc_l << "\nXc_r= " << Xc_r << "\nXt_lb= " << Xt_lb << "\nXt_rb= " << Xt_rb << "\nXt_lt= " << Xt_lt << "\nXt_rt= " << Xt_rt << "\np_lb= " << p_lb << "\np_rb= " << p_rb << "\np_lt= " << p_lt << "\np_rt= " << p_rt << "\np_lb_ind= " << p_lb_ind << "\np_rb_ind= " << p_rb_ind << "\np_lt_ind= " << p_lt_ind << "\np_rt_ind= " << p_rt_ind);
      
      
      //cv::Rect col = getColumn(image_ref_,p_lt,p_lb);
      
      //std::cout << "p_lb = " << p_lb << ", p_lt = " << p_lt << "\noriginal method:\np_lb_ind=" << p_lb_ind << "\np_lt_ind=" << p_lt_ind << "\nnew method:\n" << col << "\n";
      
      //col = getColumn(image_ref_,p_rt,p_rb);
      //std::cout << "p_rb = " << p_rb << ", p_rt = " << p_rt << "\noriginal method:\np_rb_ind=" << p_rb_ind << "\np_rt_ind=" << p_rt_ind << "\nnew method:\n" << col << "\n";
      
      if(p_lb.x > 0)
      {
        left = p_lb_ind.x + 1;
        //float depth = Xt_lb.z*scale_;
        float depth = cam_model_->getPixelValue(Xt_lb);
        
        cols.push_back(getColumn(p_lt,p_lb,depth,img_width, img_height));

      }
      
      
      //Right column
      if(p_rb.x < img_width-1)
      {
        right = p_rb_ind.x;
        //float depth = Xt_rb.z*scale_;
        float depth = cam_model_->getPixelValue(Xt_rb);
        
        cols.push_back(getColumn(p_rt,p_rb,depth,img_width, img_height));

      }
    }


      if(left >= right)
      {
        ROS_DEBUG_STREAM_NAMED(name_, "somethings wrong! " << pt);
      }
    
    for(int p_x : cam_model_->getColumnRange(left,right))
    {
        
      /*%reprojecting x pixel coordinate at center of y coordinates to ray
      L = K\[p_x;K(2,3);1];
      %%Equivalent to the below 
      %L = K\p_reproj;    %reprojecting pixel to ray
      %L(2) = 0;
      */
      
      cv::Point3d ray = cam_model_->projectPixelTo3dRay(cv::Point2d(p_x,pt.y)); //I think that the pt.y can be 0 without effect
      //ray.y = 0;
      
      
      //equations for intersection of ray and circle
      double a = ray.x*ray.x + ray.z*ray.z;
      double b = -2*(ray.x*pt.x + ray.z*pt.z);
      double c = h_squared - radius_*radius_;
      
      if(b*b - 4*a*c <0)
      {
        ROS_ERROR_STREAM_NAMED(name_, "complex solution! Left=" << left << ", right=" << right << ", p_x="<< p_x << ", ray=" << ray << ", h_squared="<< h_squared );
        continue;
        //ROS_BREAK();
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED(name_, "Left=" << left << ", right=" << right << ", p_x="<< p_x << ", ray=" << ray << ", h_squared="<< h_squared );
      }
      
      ROS_ASSERT_MSG(b*b - 4*a*c >=0, "Complex solution for ray-circle intersection!");

      /*
      Solve for parameter t that yields intersection
      Note that we only care about the more distant intersection (the + solution)
      */
      double t = (-b + std::sqrt(b*b-4*a*c))/(2*a);
      
      //Get world coordinates of intersection
      cv::Point3d X_h = ray*t;
      X_h.y = pt.y;
      
      //for bottom:
      cv::Point3d X_hb = X_h + cv::Point3d(0,height_/2,0);
      
      //for top:
      cv::Point3d X_ht = X_h + cv::Point3d(0,-height_/2,0);
      
      //project back to pixels to get y coordinate
      cv::Point2d p_xhb =  cam_model_->project3dToPixel(X_hb);
      cv::Point2d p_xht =  cam_model_->project3dToPixel(X_ht);

      //float depth = X_h.z*scale_;     
      float depth = cam_model_->getPixelValue(X_h);
      cols.push_back(getColumn(p_xht,p_xhb,depth,img_width, img_height));
      
    }
    
    
    return cols;
  }
  
};

    }
  }
}
  

#endif


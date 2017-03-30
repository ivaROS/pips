
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

  
  void CylindricalModel::setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im)
  {   
      robot_radius_ = radius + safety_expansion;
      robot_height_ = height;
      floor_tolerance_ = floor_tolerance;
      
      show_im_ = show_im;
  }
  
  // Should use my custom comparison code from RectangularModel, make it more general and move it somewhere (maybe a header?), then use it from both of these
  
  // Question: is it better to use the camera model's methods for clarity, or to use pure Eigen matrices for speed?
  // Initially, will use the model, but will likely switch over in the future.
  bool CylindricalModel::testCollision(const cv::Point3d pt)
  {
    
    cv::Mat viz;
    if(show_im_)
    {
        viz = image_ref_.clone();
    }
    bool collided = false;
    
    unsigned int left = 0;
    unsigned int right = image_ref_.cols;
    
    double h_squared = pt.x*pt.x + pt.z*pt.z;
    double h = std::sqrt(h_squared);
    
    // We only calculate the actual side borders of the robot if it is far enough away that they could be seen
    if(h > robot_radius_)
    {
      double tangentDist = std::sqrt(h_squared - robot_radius_*robot_radius_);
      
      double theta_c = std::atan2(pt.x,pt.z);
      double theta_d = std::asin(robot_radius_/h);
      
      cv::Point3d Xc_l(tangentDist*std::sin(theta_c - theta_d), pt.y, tangentDist*std::cos(theta_c - theta_d));
      cv::Point3d Xc_r(tangentDist*std::sin(theta_c + theta_d), pt.y, tangentDist*std::cos(theta_c + theta_d));

      cv::Point3d Xt_lb = Xc_l + cv::Point3d(0,-floor_tolerance_,0);
      cv::Point3d Xt_rb = Xc_r + cv::Point3d(0,-floor_tolerance_,0);

      cv::Point3d Xt_lt = Xc_l + cv::Point3d(0,-robot_height_,0);
      cv::Point3d Xt_rt = Xc_r + cv::Point3d(0,-robot_height_,0);

      cv::Point2d p_lb = cam_model_->project3dToPixel(Xt_lb);
      cv::Point2d p_rb = cam_model_->project3dToPixel(Xt_rb);
      cv::Point2d p_lt = cam_model_->project3dToPixel(Xt_lt);
      cv::Point2d p_rt = cam_model_->project3dToPixel(Xt_rt);
      
      //std::cout << "pt= " << pt << "\nh_squared= " << h_squared << "\nh= " << h << "\ntheta_c= " << theta_c << "\ntheta_d= " << theta_d << "\nXt_lb= " << Xt_lb << "\nXt_rb= " << Xt_rb << "\n";

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
      
      ROS_DEBUG_STREAM("pt= " << pt << "\nh_squared= " << h_squared << "\nh= " << h << "\ntheta_c= " << theta_c << "\ntheta_d= " << theta_d << "\nXt_lb= " << Xt_lb << "\nXt_rb= " << Xt_rb << "\nXt_lt= " << Xt_lt << "\nXt_rt= " << Xt_rt << "\np_lb= " << p_lb << "\np_rb= " << p_rb << "\np_lt= " << p_lt << "\np_rt= " << p_rt << "\np_lb_ind= " << p_lb_ind << "\np_rb_ind= " << p_rb_ind << "\np_lt_ind= " << p_lt_ind << "\np_rt_ind= " << p_rt_ind);
      
      
      //cv::Rect col = getColumn(image_ref_,p_lt,p_lb);
      
      //std::cout << "p_lb = " << p_lb << ", p_lt = " << p_lt << "\noriginal method:\np_lb_ind=" << p_lb_ind << "\np_lt_ind=" << p_lt_ind << "\nnew method:\n" << col << "\n";
      
      //col = getColumn(image_ref_,p_rt,p_rb);
      //std::cout << "p_rb = " << p_rb << ", p_rt = " << p_rt << "\noriginal method:\np_rb_ind=" << p_rb_ind << "\np_rt_ind=" << p_rt_ind << "\nnew method:\n" << col << "\n";
      
      if(p_lb.x > 0)
      {
        left = p_lb_ind.x + 1;
        cv::Rect col = getColumn(image_ref_,p_lt,p_lb);
        double depth = Xt_lb.z*scale_;

        /*Note: I should write my own matrix-matrix/matrix-scalar method that returns boolean as soon as condition satisfied*/

        if(show_im_)
        {
          cv::Mat(viz,col) = depth;
        }
        //std::cout << "lcol value = " << X_h.z*scale_ "\n";
        if(cv::countNonZero(cv::Mat(image_ref_,col) < depth) > 0)
        {
          if(show_im_)
          {
              collided = true;
          }
          else
          {
              return true;
          }
        }
        
      }
      
      
      //Right column
      if(p_rb.x < image_ref_.cols-1)
      {
        right = p_rb_ind.x;
        cv::Rect col = getColumn(image_ref_,p_rt,p_rb);
        double depth = Xt_rb.z*scale_;
        if(show_im_)
        {
          cv::Mat(viz,col) = depth;
        }
        //viz.col(p_rt_ind.x).rowRange(p_rt_ind.y,p_rb_ind.y) = Xt_rb.z*scale_;
        if(cv::countNonZero(cv::Mat(image_ref_,col) < depth) > 0)
        {
          if(show_im_)
          {

              collided = true;
          }
          else
          {
              return true;
          }
        }
        

      }
    }


    for(unsigned int p_x = left; p_x < right; ++p_x)
    {
        
      /*%reprojecting x pixel coordinate at center of y coordinates to ray
      L = K\[p_x;K(2,3);1];
      %%Equivalent to the below 
      %L = K\p_reproj;    %reprojecting pixel to ray
      %L(2) = 0;
      */
      
      cv::Point3d ray = cam_model_->projectPixelTo3dRay(cv::Point2d(p_x,pt.y));
      //ray.y = 0;
      
      
      //equations for intersection of ray and circle
      double a = ray.x*ray.x + ray.z*ray.z;
      double b = -2*(ray.x*pt.x + ray.z*pt.z);
      double c = h_squared - robot_radius_*robot_radius_;
      
      if(b*b - 4*a*c <0)
      {
        ROS_ERROR_STREAM_NAMED(name_, "complex solution! Left=" << left << ", right=" << right << ", p_x="<< p_x << ", ray=" << ray << ", h_squared="<< h_squared );
        ROS_BREAK();
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
      cv::Point3d X_hb = X_h + cv::Point3d(0,-floor_tolerance_,0);
      
      //for top:
      cv::Point3d X_ht = X_h + cv::Point3d(0,-robot_height_,0);
      
      //project back to pixels to get y coordinate
      cv::Point2d p_xhb =  cam_model_->project3dToPixel(X_hb);
      cv::Point2d p_xht =  cam_model_->project3dToPixel(X_ht);
      
      cv::Rect column = getColumn(image_ref_, p_xht,p_xhb);
      double depth = X_h.z*scale_;
      if(show_im_)
      {
            cv::Mat(viz,column) = depth;
      }
      
      if(cv::countNonZero(cv::Mat(image_ref_,column) < depth)>0)
      {
        if(show_im_)
        {
            collided = true;
        }
        else
        {
            return true;
        }
      }
      
    }
    
    if(show_im_)
    {
      double min;
      double max;
      cv::minMaxIdx(viz, &min, &max);
      cv::Mat adjMap;
      cv::convertScaleAbs(viz, viz, 255 / max);
      
      cv::imshow("Visualization", viz);
      cv::waitKey(0);
      //return false;
    }
    return collided;
  }

  /* This is basically a duplicate of the above function with some extra code added. I feel like
  a lot of this could be cleaned up into separate functions, but this will work for now */
  cv::Mat CylindricalModel::generateHallucinatedRobot(const cv::Point3d pt)
  {
    
    cv::Mat viz = cv::Mat::zeros(image_ref_.rows, image_ref_.cols, image_ref_.type()); //image_ref_.clone();
    
    unsigned int left = 0;
    unsigned int right = image_ref_.cols;
    
    double h_squared = pt.x*pt.x + pt.z*pt.z;
    double h = std::sqrt(h_squared);
    
    // We only calculate the actual side borders of the robot if it is far enough away that they could be seen
    if(h > robot_radius_)
    {
      
      double theta_c = std::atan2(pt.x,pt.z);
      double theta_d = std::asin(robot_radius_/h);
      
      cv::Point3d Xc_l(h*std::sin(theta_c - theta_d), pt.y, h*std::cos(theta_c - theta_d));
      cv::Point3d Xc_r(h*std::sin(theta_c + theta_d), pt.y, h*std::cos(theta_c + theta_d));

      cv::Point3d Xt_lb = Xc_l + cv::Point3d(0,-floor_tolerance_,0);
      cv::Point3d Xt_rb = Xc_r + cv::Point3d(0,-floor_tolerance_,0);

      cv::Point3d Xt_lt = Xc_l + cv::Point3d(0,-robot_height_,0);
      cv::Point3d Xt_rt = Xc_r + cv::Point3d(0,-robot_height_,0);

      cv::Point2d p_lb = cam_model_->project3dToPixel(Xt_lb);
      cv::Point2d p_rb = cam_model_->project3dToPixel(Xt_rb);
      cv::Point2d p_lt = cam_model_->project3dToPixel(Xt_lt);
      cv::Point2d p_rt = cam_model_->project3dToPixel(Xt_rt);
      
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
      
      ROS_DEBUG_STREAM_NAMED(name_, "pt= " << pt << "\nh_squared= " << h_squared << "\nh= " << h << "\ntheta_c= " << theta_c << "\ntheta_d= " << theta_d << "\nXt_lb= " << Xt_lb << "\nXt_rb= " << Xt_rb << "\nXt_lt= " << Xt_lt << "\nXt_rt= " << Xt_rt << "\np_lb= " << p_lb << "\np_rb= " << p_rb << "\np_lt= " << p_lt << "\np_rt= " << p_rt << "\np_lb_ind= " << p_lb_ind << "\np_rb_ind= " << p_rb_ind << "\np_lt_ind= " << p_lt_ind << "\np_rt_ind= " << p_rt_ind);
      
      // Catch cases where completely off screen
      if(p_lb.x > image_ref_.cols -1 || p_rb.x < 0)
        return viz;
      
      //left column
      if(p_lb.x > 0)
      {
        left = p_lb_ind.x + 1;
        cv::Rect col = getColumn(image_ref_,p_lt,p_lb);
        double depth = Xt_lb.z*scale_;
        cv::Mat(viz,col) = depth;
      }
      
      //Right column
      if(p_rb.x < image_ref_.cols-1)
      {
        right = p_rb_ind.x - 1;
        cv::Rect col = getColumn(image_ref_,p_rt,p_rb);
        double depth = Xt_rb.z*scale_;
        cv::Mat(viz,col) = depth;
      }
    }



      
    for(unsigned int p_x = left; p_x < right; ++p_x)
    {

      cv::Point3d ray = cam_model_->projectPixelTo3dRay(cv::Point2d(p_x,pt.y));
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
      X_h.y = pt.y;
      
      //for bottom:
      cv::Point3d X_hb = X_h + cv::Point3d(0,-floor_tolerance_,0);
      
      //for top:
      cv::Point3d X_ht = X_h + cv::Point3d(0,-robot_height_,0);
      
      //project back to pixels to get y coordinate
      cv::Point2d p_xhb =  cam_model_->project3dToPixel(X_hb);
      cv::Point2d p_xht =  cam_model_->project3dToPixel(X_ht);
      
      cv::Rect column = getColumn(image_ref_, p_xht,p_xhb);
      double depth = X_h.z*scale_;
      cv::Mat(viz,column) = depth; 

    }

    return viz;
  }


cv::Rect CylindricalModel::getColumn(const cv::Mat& image, const cv::Point2d& top, const cv::Point2d& bottom)
{
    //By forming a Rect in this way, doesn't matter which point is the top and which is the bottom.
    cv::Rect_<double> r(top,bottom);
    
    int x = r.tl().x + .00000000001;  // Add a tiny number to handle cases where the process of projecting to ray, finding intersection, and projecting back to the image plane introduces a tiny numerical error, apparently only with smaller values that are odd. The added value will never push the number to the next integer value but is much larger than any error seen so far
    int y = std::floor(r.tl().y);
    int width = 1;
    int height = std::ceil(r.br().y)-y + 1; //ROI's from rectangles are noninclusive on the right/bottom sides, so need to add 1 to include the bottom row
    
    cv::Rect column(y,x,height,width);
    cv::Rect imageBounds(0,0,image.cols,image.rows);
    cv::Rect bounded = column & imageBounds;

    
    //std::cout << std::setprecision(16) << "top = " << top << ", bottom = " << bottom << ", Col = " << r << "tl: " << r.tl() << " br: " << r.br() << ", rect=" << column << ", bounded= " << bounded << "\n"; // Debugging printout to troubleshoot incorrect x values for columns. The 'magic number' added above resolved the problem.
    
    return bounded;
}

cv::Mat CylindricalModel::getImage(cv_bridge::CvImage::ConstPtr& cv_image_ref)
{
   cv::Mat transposed = cv_image_ref->image.t();
   return transposed;
}
    



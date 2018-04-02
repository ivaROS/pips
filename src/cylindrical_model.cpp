
#include "pips/collision_testing/robot_models/cylindrical_model.h"
#include <pips/utils/image_comparison_implementations.h>
//#include <sensor_msgs/Image.h>
//#include <geometry_msgs/TransformStamped.h>

//#include <opencv/cv.h>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"


//#include <Eigen/Eigen>
//#include <image_transport/image_transport.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <cv_bridge/cv_bridge.h>

//#include <iomanip>      // std::setprecision

#include "ros/ros.h" //Needed for 'ROS_BREAK and ROS_ASSERT

    CylindricalModel::CylindricalModel() : HallucinatedRobotModelImpl<cv::Point3d>() 
    {
      this->name_ = "CylindricalModel";
    }
    
    
  void CylindricalModel::setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im)
  {   
      robot_radius_ = radius + safety_expansion;
      robot_height_ = height;
      floor_tolerance_ = floor_tolerance;
      
      show_im_ = show_im;
  }
  



  cv::Mat CylindricalModel::generateHallucinatedRobotImpl(const cv::Point3d pt)
  {
    cv::Mat viz = HallucinatedRobotModelImpl::generateHallucinatedRobotImpl(pt);
    
    std::vector<COLUMN_TYPE> cols = getColumns(pt);
    
    for(unsigned int i = 0; i < cols.size(); ++i)
    {
      cv::Mat col = cv::Mat(viz, cols[i].rect); //cols[i].image;
      float depth = cols[i].depth;
      
      col.setTo(depth);
    }
    
    return viz;
  }



  COLUMN_TYPE CylindricalModel::getColumn(const cv::Point2d top, const cv::Point2d bottom, const float depth)
  {
    //By forming a Rect in this way, doesn't matter which point is the top and which is the bottom.
    cv::Rect_<double> r(top,bottom);
    
    int x = r.tl().x + .00000000001;  // Add a tiny number to handle cases where the process of projecting to ray, finding intersection, and projecting back to the image plane introduces a tiny numerical error, apparently only with smaller values that are odd. The added value will never push the number to the next integer value but is much larger than any error seen so far
    int y = std::floor(r.tl().y);
    int width = 1;
    int height = std::ceil(r.br().y)-y + 1; //ROI's from rectangles are noninclusive on the right/bottom sides, so need to add 1 to include the bottom row
    
  //The only changes needed to use a transposed image are swapping the x and y as well as width and height
    cv::Rect column = getColumnRect(x,y,width,height);
    cv::Rect imageBounds(0,0,image_ref_.cols,image_ref_.rows);
    cv::Rect bounded = column & imageBounds;

    COLUMN_TYPE col;
    
    col.rect = bounded;
    //col.image = cv::Mat(image_ref_,bounded);
    col.depth = depth;

     ROS_DEBUG_STREAM_NAMED(name_, "Input rect: " << r << ", bounded: " << bounded << ", depth: " << depth);
     
    //std::cout << std::setprecision(16) << "top = " << top << ", bottom = " << bottom << ", Col = " << r << "tl: " << r.tl() << " br: " << r.br() << ", rect=" << column << ", bounded= " << bounded << "\n"; // Debugging printout to troubleshoot incorrect x values for columns. The 'magic number' added above resolved the problem.
    
    return col;
  }

  cv::Rect CylindricalModel::getColumnRect(const int x, const int y, const int width, const int height)
  {
      return cv::Rect(x,y,width,height);
  }

  /*
  void doPrecomputation(cv_bridge::CvImage::ConstPtr& cv_image_ref) 
  {
    img_width_ = cv_image_ref->image.cols;
    img_height_ = cv_image_ref->image.rows;
  }
  */
  
  cv::Rect CylindricalModel::getROIImpl(const cv::Point3d pt)
  {
    cv::Rect rect;
    
    //It would probably be more efficient to just grab the code that computes the corners, but this should work
    std::vector<COLUMN_TYPE> cols = getColumns(pt);
    
    for(unsigned int i = 0; i < cols.size(); ++i)
    {
      rect |= cols[i].rect;
    }
    
    ROS_INFO_STREAM_THROTTLE(1,"Pt = " << pt << ", rect = " << rect);
    
    return rect;
  }

  bool CylindricalModel::inFrame(const cv::Point3d& pt)
  {
    double h_squared = pt.x*pt.x + pt.z*pt.z;
    double h = std::sqrt(h_squared);
  
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
    
    cv::Rect frame = getImageRect();
    if(frame.contains(p_lb) && frame.contains(p_rb) && frame.contains(p_lt) && frame.contains(p_rt))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  std::vector<COLUMN_TYPE> CylindricalModel::getColumns(const cv::Point3d pt)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Get columns for " << pt);
    std::vector<COLUMN_TYPE> cols;
    
    int img_width = cv_image_ref_->image.cols;
    int img_height = cv_image_ref_->image.rows;
    
    unsigned int left = 0;
    unsigned int right = img_width;
    
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
      
      ROS_DEBUG_STREAM("pt= " << pt << "\nh_squared= " << h_squared << "\nh= " << h << "\ntheta_c= " << theta_c << "\ntheta_d= " << theta_d << "\nXt_lb= " << Xt_lb << "\nXt_rb= " << Xt_rb << "\nXt_lt= " << Xt_lt << "\nXt_rt= " << Xt_rt << "\np_lb= " << p_lb << "\np_rb= " << p_rb << "\np_lt= " << p_lt << "\np_rt= " << p_rt << "\np_lb_ind= " << p_lb_ind << "\np_rb_ind= " << p_rb_ind << "\np_lt_ind= " << p_lt_ind << "\np_rt_ind= " << p_rt_ind);
      
      
      //cv::Rect col = getColumn(image_ref_,p_lt,p_lb);
      
      //std::cout << "p_lb = " << p_lb << ", p_lt = " << p_lt << "\noriginal method:\np_lb_ind=" << p_lb_ind << "\np_lt_ind=" << p_lt_ind << "\nnew method:\n" << col << "\n";
      
      //col = getColumn(image_ref_,p_rt,p_rb);
      //std::cout << "p_rb = " << p_rb << ", p_rt = " << p_rt << "\noriginal method:\np_rb_ind=" << p_rb_ind << "\np_rt_ind=" << p_rt_ind << "\nnew method:\n" << col << "\n";
      
      if(p_lb.x > 0)
      {
        left = p_lb_ind.x + 1;
        float depth = Xt_lb.z*scale_;
        cols.push_back(getColumn(p_lt,p_lb,depth));

      }
      
      
      //Right column
      if(p_rb.x < img_width-1)
      {
        right = p_rb_ind.x;
        float depth = Xt_rb.z*scale_;
        cols.push_back(getColumn(p_rt,p_rb,depth));

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

      float depth = X_h.z*scale_;      
      cols.push_back(getColumn(p_xht,p_xhb,depth));
      
    }
    
    
    return cols;
  }
  
  ComparisonResult CylindricalModel::testCollisionImpl(const cv::Point3d pt, CCOptions options)
  {
    std::vector<COLUMN_TYPE> cols = getColumns(pt);
  
    ComparisonResult result;
    for(unsigned int i = 0; i < cols.size(); ++i)
    {
      cv::Mat col = cv::Mat(this->image_ref_,cols[i].rect); //cols[i].image;
      float depth = cols[i].depth;
      
      ComparisonResult column_result = isLessThan(col, depth);

      if(column_result && options)
      {        
        if(!column_result.hasDetails())
        {
            column_result = isLessThanDetails(col,depth);
        }
        cv::Point offset;
        cv::Size size;
        col.locateROI(size, offset);
            
        column_result.addOffset(offset);	

        if(show_im_)
        {
          result.addResult(column_result);
        }
        else
        {
          return column_result;
        }
      }
    }
    
    
 
    return result;
  }
  
  ComparisonResult CylindricalModel::isLessThan(const cv::Mat& col, float depth)
  {
    return utils::isLessThan::stock(col, depth);
  }
  
  ComparisonResult CylindricalModel::isLessThanDetails(const cv::Mat& col, float depth)
  {
    if(show_im_)
    {
      ROS_INFO_STREAM("FULL details!");
      return utils::isLessThan::fulldetails(col, depth);
    }
    else
    {
      return utils::isLessThan::details(col, depth);
    }
  }
    



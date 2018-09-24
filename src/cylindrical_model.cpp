
#include <pips/collision_testing/robot_models/cylindrical_model.h>
#include <pips/utils/image_comparison_implementations.h>
//#include <sensor_msgs/Image.h>
//#include <geometry_msgs/TransformStamped.h>

//#include <opencv/cv.h>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
      robot_height_ = height-floor_tolerance;
      floor_tolerance_ = floor_tolerance;
      
      show_im_ = show_im;
      
      geometry_msgs::TransformStamped origin;
      origin.transform.translation.z = robot_height_/2 + floor_tolerance;
      origin.transform.rotation.w = 1;
      
      cylinder_ = pips::collision_testing::geometry_models::Cylinder(robot_radius_, robot_height_);
      cylinder_.cam_model_ = cam_model_;
      cylinder_.setOrigin(origin);
      tf2::doTransform(cylinder_.origin_transform_, cylinder_.current_transform_, base_optical_transform_); //TODO: don't make so many assumptions
      
  }
  

  geometry_msgs::Pose CylindricalModel::transformPose(const geometry_msgs::Pose& pose)
  {
    Eigen::Affine3d pose_eig;
    tf2::fromMsg(pose, pose_eig);
    
    //Transform coordinates of robot base into camera's optical frame
    Eigen::Affine3d pose_eig_t;
    
    tf2::doTransform(pose_eig, pose_eig_t, cylinder_.current_transform_);
    
    geometry_msgs::Pose pose_t;
    convertPose(pose_eig_t, pose_t);

    
    ROS_DEBUG_STREAM_NAMED(name_, "Pose " << toString(pose) << " transformed to " << toString(pose_t) );
    
    return pose_t;
  }


  cv::Mat CylindricalModel::generateHallucinatedRobotImpl(const cv::Point3d pt)
  {
    cv::Mat viz = HallucinatedRobotModelImpl::generateHallucinatedRobotImpl(pt);
    
    std::vector<COLUMN_TYPE> cols = getColumns(pt);
    
    for(unsigned int i = 0; i < cols.size(); ++i)
    {
      cv::Rect roi = getColumnRect(cols[i].rect);
      cv::Mat col = cv::Mat(viz, roi); //cols[i].image;
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

  cv::Rect CylindricalModel::getColumnRect(const cv::Rect& rect)
  {
      return getColumnRect(rect.tl().x,rect.tl().y,rect.width,rect.height);
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
    
    ROS_DEBUG_STREAM_THROTTLE(1,"Pt = " << pt << ", rect = " << rect);
    
    return rect;
  }

  bool CylindricalModel::inFrame(const cv::Point3d& pt)
  {
    // TODO: consolidate repeated code into functions
    
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
  
  
  void CylindricalModel::getIntersection(cv::Point3d pt, double r, cv::Point3d& left, cv::Point3d& right)
  {
    cv::Point3d robot_origin;
    
    //https://stackoverflow.com/a/42803692/2906021
    double d = std::sqrt((pt.x - robot_origin.x)*(pt.x - robot_origin.x)  +  (pt.y - robot_origin.y)*(pt.y - robot_origin.y));
    double a = (robot_radius_*robot_radius_ - r*r + d*d)/(2*d);
    double h = std::sqrt(robot_radius_*robot_radius_ - a*a);
    double x2 = robot_origin.x + a*(pt.x-robot_origin.x)/d;
    double y2 = robot_origin.y + a*(pt.y-robot_origin.y)/d;
    
    right.x = x2 + h*(pt.y-robot_origin.y)/d;
    right.y = y2 - h*(pt.x-robot_origin.x)/d;
    
    left.x = x2 - h*(pt.y-robot_origin.y)/d;
    left.y = y2 + h*(pt.x-robot_origin.x)/d;
    
    
  }
  
  

  std::vector<COLUMN_TYPE> CylindricalModel::getColumns(const cv::Point3d pt)
  {
    return cylinder_.getColumns(pt, cv_image_ref_->image.cols, cv_image_ref_->image.rows);
  }
  
  ComparisonResult CylindricalModel::testCollisionImpl(const cv::Point3d pt, CCOptions options)
  {
    std::vector<COLUMN_TYPE> cols = getColumns(pt);
  
    ComparisonResult result;
    for(unsigned int i = 0; i < cols.size(); ++i)
    {
      //TODO: limit rect to image size
      cv::Rect roi = getColumnRect(cols[i].rect);
      cv::Mat col = cv::Mat(this->image_ref_,roi); //cols[i].image;
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
    // TODO: replace 'show_im_' with more accurate variable name (ex: 'full_details' or something)
    if(show_im_)
    {
      ROS_DEBUG_STREAM_NAMED(name_,"FULL details!");
      return utils::isLessThan::fulldetails(col, depth);
    }
    else
    {
      return utils::isLessThan::details(col, depth);
    }
  }
    



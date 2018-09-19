
#include <pips/collision_testing/robot_models/box_model.h>
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

    BoxModel::BoxModel() : HallucinatedRobotModelImpl<geometry_msgs::Pose>() 
    {
      this->name_ = "BoxModel";
    }
    
//TODO this goes back to HallucinatedRobotModelBase for paramete setting
  void BoxModel::setParameters(double width, double height, double length, double rear_distance, double safety_expansion, double floor_tolerance, bool show_im)
  {   
      robot_radius_ = width + safety_expansion;
      robot_height_ = height;
      robot_length = length + safety_expansion;
      floor_tolerance_ = floor_tolerance;
      distance_from_rear = rear_distance;
      
      show_im_ = show_im;
  }
  
    void BoxModel::setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im)
    {
        //TODO Hard codding is the option used now(the two double variable)
        //Pioneer dimentions: length = 455, width = 381, height = 237. all dimentions in mm.
        double robo_length = 0.455;
        double dist_from_rare  = robo_length/2; // for now, mid way
        setParameters(radius, height, robo_length, dist_from_rare, safety_expansion, floor_tolerance, show_im);
    }


  cv::Mat BoxModel::generateHallucinatedRobotImpl(const geometry_msgs::Pose pt)
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



  COLUMN_TYPE BoxModel::getColumn(const cv::Point2d top, const cv::Point2d bottom, const float depth)
  {
    //By forming a Rect in this way, doesn't matter which point is the top and which is the bottom.
    cv::Rect_<double> r(top,bottom);
    
    int x = r.tl().x + .00000000001;  // Add a tiny number to handle cases where the process of projecting to ray, finding intersection, and projecting back to the image plane introduces a tiny numerical error, apparently only with smaller values that are odd. The added value will never push the number to the next integer value but is much larger than any error seen so far
    int y = std::floor(r.tl().y);
    int width = 1;
    int height = std::ceil(r.br().y)-y + 1; //ROI's from rectangles are noninclusive on the right/bottom sides, so need to add 1 to include the bottom row
    
  //The only changes needed to use a transposed image are swapping the x and y as well as width and height
    cv::Rect column = cv::Rect(x,y,width,height);
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

  
  std::vector<COLUMN_TYPE> BoxModel::getColumns(const geometry_msgs::Pose pose)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Get columns for " << pose);
    std::vector<COLUMN_TYPE> cols;
    
    int img_width = cv_image_ref_->image.cols;
    //int img_height = cv_image_ref_->image.rows;
    
    //TODO make it work without this or using class parameters
    double robot_length, distance_from_rear, hrw, rd, fd;//hrl is half of the robot length, hrw is half of the robot width, and fd and rd are distance from front and rear
    
    robot_length = 0.455; //TODO make it a class variable.
    distance_from_rear = robot_length/2; // half way for now.
    
    rd = distance_from_rear;
    //hrl = robot_length/2;
    hrw = robot_radius_/2;
    fd = robot_length - rd;
    
    
    unsigned int left = 0;
    unsigned int right = img_width;
    
     cv::Point3d pt;
     convertPose(pose, pt);//TODO do I have the neccessary include files.
     
    double h_squared = pt.x*pt.x + pt.z*pt.z;
    
    double h = std::sqrt(h_squared);
    
   
    cv::Point3d pt2 = BoxModel::quaternionToRPY(pose.orientation);
    double ang = pt2.y; //TODO the pitch is what is needed(check for correctness)
    
    ROS_DEBUG_STREAM("raw, pithc, and yaw" << pt2);
    
    double theta1 = std::atan2(fd,hrw); 
    double theta2 = std::atan2(rd,hrw); 
    
    double t1, t2, t3, t4, t5, hd1, hd2;
    
    t1 = theta1 + ang;
    t2 = M_PI - theta1 + ang;
    t3 = M_PI + theta2 + ang;
    t4 = 2*M_PI - theta2 + ang;
    t5 = std::atan2(pt.x, pt.z);
    
//    corner 2.                      corner 1.
//     
//     
//     
//     
//     corner 3.                     corner 4.
    
    //hd is the hypotenuse-distance from the base frame to the corners of the car
    
    hd1  = std::sqrt(fd*fd + hrw*hrw);// from the base frame to corner 1 and 2
    hd2  = std::sqrt(rd*rd + hrw*hrw);// from the base frame to corner 3 and 4
    
    //TODO this distance is to check if the camera is not in the robot. May not be needed or may need aditional distance to be added.
    double distance = rd/std::cos(t5);   
    
    cv::Point3d p1, p2, p3, p4, p5, p6, p7, p8;
    
    // We only calculate the actual side borders of the robot if it is far enough away that they could be seen.
    if(h > distance)
    {
        //TODO remove the debug message
        ROS_DEBUG_STREAM("H was greater!!" << h);
        
        //corner 1
        p1.x = pt.x + hd1*std::cos(t1); 
        p1.z = pt.z + hd1*std::sin(t1);
        p1.y = pt.y;
        p5 =  p1 + cv::Point3d(0,-robot_height_,0);
        p1 =  p1 + cv::Point3d(0,-floor_tolerance_,0);
        
        //corner 2
        p2.x = pt.x + hd1*std::cos(t2); 
        p2.z = pt.z + hd1*std::sin(t2);
        p2.y = pt.y;
        p6 =  p2 + cv::Point3d(0,-robot_height_,0);
        p2 =  p2 + cv::Point3d(0,-floor_tolerance_,0);
        
        //corner 3
        p3.x = pt.x + hd2*std::cos(t3); 
        p3.z = pt.z + hd2*std::sin(t3);
        p3.y = pt.y;
        p7 =  p3 + cv::Point3d(0,-robot_height_,0);
        p3 =  p3 + cv::Point3d(0,-floor_tolerance_,0);
        
        //corner 4
        p4.x = pt.x + hd2*std::cos(t4); 
        p4.z = pt.z + hd2*std::sin(t4);
        p4.y = pt.y;
        p8 =  p4 + cv::Point3d(0,-robot_height_,0);
        p4 =  p4 + cv::Point3d(0,-floor_tolerance_,0);

    }
    else
    {
      return cols;
      //getIntersection(pt, robot_radius_, Xc_l, Xc_r);
    }
    
    cv::Point2d p1p, p2p, p3p, p4p, p5p, p6p, p7p, p8p;
      
    p1p = cam_model_->project3dToPixel(p1);
    p2p = cam_model_->project3dToPixel(p2);
    p3p = cam_model_->project3dToPixel(p3);
    p4p = cam_model_->project3dToPixel(p4);
    p5p = cam_model_->project3dToPixel(p5);
    p6p = cam_model_->project3dToPixel(p6);
    p7p = cam_model_->project3dToPixel(p7);
    p8p = cam_model_->project3dToPixel(p8);
    
    bool s12, s23, s34, s41, check;
    
    s12 = p1p.x > p2p.x;
    s23 = p2p.x > p3p.x;
    s34 = p3p.x > p4p.x;
    s41 = p4p.x > p1p.x;
    
    
    //keep these varibales for now
    //TODO for now calculate the left and right variables directly.
//     unsigned int p1px,p2px,p3px, p4px, p5px, p6px, p7px, p8px;
//     unsigned int p1py,p2py,p3py, p4py, p5py, p6py, p7py, p8py;

     if (s12)
    {
    //TODO remove the debug message
        check = left<=right;
        ROS_DEBUG_STREAM("side 12 is checked!!"<< check );
      //left column  
        if(p2p.x > 0)
        {
            left = std::min(std::max(0,(int)std::floor(p2p.x)),img_width-1) + 1;
            float depth = cam_model_->getPixelValue(p2) * scale_;
            cols.push_back(getColumn(p6p,p2p,depth));
        }
        //Right column
        if(p1p.x < img_width-1)
        {
            right = (int)std::max(0,std::min((int)std::ceil(p1p.x),img_width-1));
            float depth = cam_model_->getPixelValue(p1) * scale_;
            cols.push_back(getColumn(p5p,p1p,depth));
        }
        //TODO remove the debug message
         check = left<=right;
       ROS_DEBUG_STREAM("side 12 is checked!!"<< check);
        
        for(int p_x : cam_model_->getColumnRange(left,right))
        {
            cv::Point2d pix1 = cv::Point2d(p_x, p2.y);
            COLUMN_TYPE col1 = getIntersection(p1, p2, p5, pix1);
            cols.push_back(col1);
        }
    }
     if (s23)
    {
        left = 0;
        right = img_width;
        //TODO remove the debug message
         check = left<=right;
          ROS_DEBUG_STREAM("side 23 is checked!!"<< check);
        //left column  
        if(p3p.x > 0)
        {
            left = (int)std::min(std::max(0,(int)std::floor(p3p.x)),img_width-1) + 1;
            float depth = cam_model_->getPixelValue(p3) * scale_;
            cols.push_back(getColumn(p7p,p3p,depth));
        }
        //Right column
        if(p2p.x < img_width-1)
        {
            right = (int)std::max(0,std::min((int)std::ceil(p2p.x),img_width-1));
            float depth = cam_model_->getPixelValue(p2) * scale_;
            cols.push_back(getColumn(p6p,p2p,depth));
        }
        //TODO remove the debug message
         check = left<=right;
          ROS_DEBUG_STREAM("side 23 is checked!!"<< check);
         for(int p_x : cam_model_->getColumnRange(left,right))
        {
            cv::Point2d pix1 = cv::Point2d(p_x, p3.y);
            COLUMN_TYPE col1 = getIntersection(p2, p3, p6, pix1);
            cols.push_back(col1);
        }
    }
     if (s34)
    {
        left = 0;
        right = img_width;
        //TODO remove the debug message
         check = left<=right;
          ROS_DEBUG_STREAM("side 34 is checked!!"<< check );
         //left column  
        if(p4p.x > 0)
        {
           left = (int)std::min(std::max(0,(int)std::floor(p4p.x)),img_width-1) + 1;
            float depth = cam_model_->getPixelValue(p4) * scale_;
            cols.push_back(getColumn(p8p,p4p,depth));
        }
        //Right column
        if(p3p.x < img_width-1)
        {
            right = (int)std::max(0,std::min((int)std::ceil(p3p.x),img_width-1));
            float depth = cam_model_->getPixelValue(p3) * scale_;
            cols.push_back(getColumn(p7p,p3p,depth));
        }
        //TODO remove the debug message
         check = left<=right;
          ROS_DEBUG_STREAM("side 34 is checked!!"<< check );
         for(int p_x : cam_model_->getColumnRange(left,right))
        {
            cv::Point2d pix1 = cv::Point2d(p_x, p4.y);
            COLUMN_TYPE col1 = getIntersection(p3, p4, p7, pix1);
            cols.push_back(col1);
        }
    }
     if (s41)
    {
        left = 0;
        right = img_width;
        //TODO remove the debug message
         check = left<=right;
         ROS_DEBUG_STREAM("side 41 is checked!!" << check);
          //left column  
        if(p1p.x > 0)
        {
           left = (int)std::min(std::max(0,(int)std::floor(p1p.x)),img_width-1) + 1;
            float depth = cam_model_->getPixelValue(p1) * scale_;
            cols.push_back(getColumn(p5p,p1p,depth));
        }
        //Right column
        if(p4p.x < img_width-1)
        {
            right = (int)std::max(0,std::min((int)std::ceil(p4p.x),img_width-1));
            float depth = cam_model_->getPixelValue(p4) * scale_;
            cols.push_back(getColumn(p8p,p4p,depth));
        }
        //TODO remove the debug message
         check = left<=right;
          ROS_DEBUG_STREAM("side 41 is checked!!"<< check );
         for(int p_x : cam_model_->getColumnRange(left,right))
        {
            cv::Point2d pix1 = cv::Point2d(p_x, p1.y);
            COLUMN_TYPE col1 = getIntersection(p4, p1, p8, pix1);
            cols.push_back(col1);
        }
    }
    
    return cols;
  }
  
  COLUMN_TYPE BoxModel::getIntersection(cv::Point3d p1, cv::Point3d p2, cv::Point3d p3, cv::Point2d pix)
  {
        cv::Point3d normal, p12, p13;
        p12 = p1 - p2;
        p13 = p1 - p3;
        
        normal.x = p12.y*p13.z - p12.z*p13.y;
        normal.y = p12.z*p13.x - p12.x*p13.z;
        normal.z = p12.x*p13.y - p12.y*p13.x;
        
        COLUMN_TYPE col;

        cv::Point3d ray = cam_model_->projectPixelTo3dRay(pix);
        //@ Justin I think that the pt.y can be 0 without effect
        
        //equations for intersection of ray and plane
        double numera = p1.x*normal.x + p1.y*normal.y + p1.z*normal.z;
        double denom = ray.x*normal.x + ray.y*normal.y + ray.z*normal.z;
        double t;
         if (denom > 1e-6)
        {
           t = numera/denom;
        }
        else
        {
            ROS_DEBUG_STREAM_NAMED(name_, "Very small denominator!"<< "p_x="<< pix.x << "ray=" << ray);
            //TODO find a better way. 
            //can I skipp this column?
            //small t means the palne and the ray are close to being parallel 
            t = 1e-6;
        }
        
        //Get world coordinates of intersection
        cv::Point3d intersection = ray*t;
        intersection.y = p1.y + floor_tolerance_;
        
        //for top:
        cv::Point3d X_ht = intersection + cv::Point3d(0,-robot_height_,0);
        
        //for bottom:
        cv::Point3d X_hb = intersection + cv::Point3d(0,-floor_tolerance_,0);
    
        //project back to pixels to get y coordinate
        cv::Point2d p_xhb =  cam_model_->project3dToPixel(X_hb);
        cv::Point2d p_xht =  cam_model_->project3dToPixel(X_ht);

        float depth = cam_model_->getPixelValue(X_hb) * scale_;
      
        col = getColumn(p_xht,p_xhb,depth);
        
        return col;
        
  }
      
  ComparisonResult BoxModel::testCollisionImpl(const geometry_msgs::Pose pose, CCOptions options)
  {
    std::vector<COLUMN_TYPE> cols = getColumns(pose);
  
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
  
    cv::Point3d BoxModel::quaternionToRPY(const geometry_msgs::Quaternion& quaternion)
    
    {
        // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(quaternion, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        double roll, pitch, yaw;
        cv::Point3d point;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        
        point.x = roll;
        point.y = pitch;
        point.z = yaw;

        return point;
    }


  ComparisonResult BoxModel::isLessThan(const cv::Mat& col, float depth)
  {
    return utils::isLessThan::stock(col, depth);
  }
  
  ComparisonResult BoxModel::isLessThanDetails(const cv::Mat& col, float depth)
  {
    // TODO: replace 'show_im_' with more accurate variable name (ex: 'full_details' or something)
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
    



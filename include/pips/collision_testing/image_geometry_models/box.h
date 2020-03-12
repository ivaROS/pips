

#ifndef PIPS_IMAGE_GEOMETRY_MODELS_BOX_H
#define PIPS_IMAGE_GEOMETRY_MODELS_BOX_H

#include <pips/utils/pose_conversions.h>
#include <pips/collision_testing/geometry_models/geometry_models.h>
#include <pips/collision_testing/geometry_models/box.h>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"


#include "ros/ros.h" //Needed for 'ROS_BREAK and ROS_ASSERT


namespace pips
{
  namespace collision_testing
  {
      
      namespace image_geometry_models
      {


  class Box : GeometryModel
  {
  public:
    double length_, width_, height_;
    
  
    static cv::Point3d quaternionToRPY(const geometry_msgs::Quaternion& quaternion)
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
    
public:
  Box(const std::shared_ptr<const pips::collision_testing::geometry_models::Box>& source, const geometry_msgs::Pose& pose):
    GeometryModel(source, pose),
    length_(source->length_),
    width_(source->width_),
    height_(source->height_)
  {}
  
  std::vector<COLUMN_TYPE> getColumns(const std::shared_ptr<const pips::utils::AbstractCameraModel>& cam_model_, int img_width, int img_height) const
  {
    ROS_ASSERT(length_>0 && width_ > 0 && height_>0);
    
    ROS_DEBUG_STREAM("Get columns for " << pose_);
    std::vector<COLUMN_TYPE> cols;
        
    //TODO make it work without this or using class parameters
    double distance_from_rear, hrw, rd, fd;//hrl is half of the robot length, hrw is half of the robot width, and fd and rd are distance from front and rear
    
    distance_from_rear = length_/2; // half way for now.
    
    rd = distance_from_rear;
    hrw = width_/2;
    fd = length_ - rd;
    
    
    unsigned int left = 0;
    unsigned int right = img_width;
    
    cv::Point3d pt;
    convertPose(pose_, pt);
     
    double h_squared = pt.x*pt.x + pt.z*pt.z;
    
    double h = std::sqrt(h_squared);
    
   
    cv::Point3d pt2 = quaternionToRPY(pose_.orientation);
    double ang = pt2.z; //Accessing yaw value since orientation is expected to not be transformed
    
    ROS_DEBUG_STREAM("roll, pitch, and yaw" << pt2);
    
    double theta1 = std::atan2(fd,hrw); 
    double theta2 = std::atan2(rd,hrw); 
    
    ROS_DEBUG_STREAM("theta1: " << theta1 << ", theta2: " << theta2);
    
    //double theta1 = std::atan2(hrw,fd); 
    //double theta2 = std::atan2(hrw,rd); 
    
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
        p5 =  p1 + cv::Point3d(0,-height_/2,0);
        p1 =  p1 + cv::Point3d(0,height_/2,0);
        
        //corner 2
        p2.x = pt.x + hd1*std::cos(t2); 
        p2.z = pt.z + hd1*std::sin(t2);
        p2.y = pt.y;
        p6 =  p2 + cv::Point3d(0,-height_/2,0);
        p2 =  p2 + cv::Point3d(0,height_/2,0);
        
        //corner 3
        p3.x = pt.x + hd2*std::cos(t3); 
        p3.z = pt.z + hd2*std::sin(t3);
        p3.y = pt.y;
        p7 =  p3 + cv::Point3d(0,-height_/2,0);
        p3 =  p3 + cv::Point3d(0,height_/2,0);
        
        //corner 4
        p4.x = pt.x + hd2*std::cos(t4); 
        p4.z = pt.z + hd2*std::sin(t4);
        p4.y = pt.y;
        p8 =  p4 + cv::Point3d(0,-height_/2,0);
        p4 =  p4 + cv::Point3d(0,height_/2,0);

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
            float depth = cam_model_->getPixelValue(p2);
            cols.push_back(getColumn(p6p,p2p,depth, img_width, img_height));
        }
        //Right column
        if(p1p.x < img_width-1)
        {
            right = (int)std::max(0,std::min((int)std::ceil(p1p.x),img_width-1));
            float depth = cam_model_->getPixelValue(p1);
            cols.push_back(getColumn(p5p,p1p,depth, img_width, img_height));
        }
        //TODO remove the debug message
         check = left<=right;
       ROS_DEBUG_STREAM("side 12 is checked!!"<< check);
        
        for(int p_x : cam_model_->getColumnRange(left,right))
        {
            cv::Point2d pix1 = cv::Point2d(p_x, p2.y);
            COLUMN_TYPE col1 = getIntersection(cam_model_, p1, p2, p5, pix1, img_width, img_height);
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
            float depth = cam_model_->getPixelValue(p3);
            cols.push_back(GeometryModel::getColumn(p7p,p3p,depth, img_width, img_height));
        }
        //Right column
        if(p2p.x < img_width-1)
        {
            right = (int)std::max(0,std::min((int)std::ceil(p2p.x),img_width-1));
            float depth = cam_model_->getPixelValue(p2);
            cols.push_back(getColumn(p6p,p2p,depth, img_width, img_height));
        }
        //TODO remove the debug message
         check = left<=right;
          ROS_DEBUG_STREAM("side 23 is checked!!"<< check);
         for(int p_x : cam_model_->getColumnRange(left,right))
        {
            cv::Point2d pix1 = cv::Point2d(p_x, p3.y);
            COLUMN_TYPE col1 = getIntersection(cam_model_, p2, p3, p6, pix1, img_width, img_height);
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
            float depth = cam_model_->getPixelValue(p4);
            cols.push_back(GeometryModel::getColumn(p8p,p4p,depth, img_width, img_height));
        }
        //Right column
        if(p3p.x < img_width-1)
        {
            right = (int)std::max(0,std::min((int)std::ceil(p3p.x),img_width-1));
            float depth = cam_model_->getPixelValue(p3);
            cols.push_back(GeometryModel::getColumn(p7p,p3p,depth, img_width, img_height));
        }
        //TODO remove the debug message
         check = left<=right;
          ROS_DEBUG_STREAM("side 34 is checked!!"<< check );
         for(int p_x : cam_model_->getColumnRange(left,right))
        {
            cv::Point2d pix1 = cv::Point2d(p_x, p4.y);
            COLUMN_TYPE col1 = getIntersection(cam_model_, p3, p4, p7, pix1, img_width, img_height);
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
            float depth = cam_model_->getPixelValue(p1);
            cols.push_back(GeometryModel::getColumn(p5p,p1p,depth, img_width, img_height));
        }
        //Right column
        if(p4p.x < img_width-1)
        {
            right = (int)std::max(0,std::min((int)std::ceil(p4p.x),img_width-1));
            float depth = cam_model_->getPixelValue(p4);
            cols.push_back(GeometryModel::getColumn(p8p,p4p,depth, img_width, img_height));
        }
        //TODO remove the debug message
         check = left<=right;
          ROS_DEBUG_STREAM("side 41 is checked!!"<< check );
         for(int p_x : cam_model_->getColumnRange(left,right))
        {
            cv::Point2d pix1 = cv::Point2d(p_x, p1.y);
            COLUMN_TYPE col1 = getIntersection(cam_model_, p4, p1, p8, pix1, img_width, img_height);
            cols.push_back(col1);
        }
    }
    
    return cols;
  }
  

  
  COLUMN_TYPE getIntersection(const std::shared_ptr<const pips::utils::AbstractCameraModel>& cam_model_, cv::Point3d p1, cv::Point3d p2, cv::Point3d p3, cv::Point2d pix, int img_width, int img_height) const
  {
        //TODO: finding the normal and passing it might be faster
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
            ROS_DEBUG_STREAM("Very small denominator!"<< "p_x="<< pix.x << "ray=" << ray);
            //TODO find a better way. 
            //can I skipp this column?
            //small t means the palne and the ray are close to being parallel 
            t = 1e-6;
        }
        
        //Get world coordinates of intersection
        cv::Point3d intersection = ray*t;
        intersection.y = p1.y;
        
        //for top:
        cv::Point3d X_ht = intersection + cv::Point3d(0,-height_,0);
        
        //for bottom:
        cv::Point3d X_hb = intersection;// + cv::Point3d(0,height_/2,0);
    
        //project back to pixels to get y coordinate
        cv::Point2d p_xhb =  cam_model_->project3dToPixel(X_hb);
        cv::Point2d p_xht =  cam_model_->project3dToPixel(X_ht);

        float depth = cam_model_->getPixelValue(X_hb);
      
        col = getColumn(p_xht,p_xhb,depth, img_width, img_height);
        
        return col;
        
  }



    };
    }
    
    }
    }
    
#endif //PIPS_IMAGE_GEOMETRY_MODELS_BOX_H

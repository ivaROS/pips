#ifndef HALLUCINATED_ROBOT_MODEL_H
#define HALLUCINATED_ROBOT_MODEL_H

#include <pips/utils/pose_conversions.h>
#include <pips/collision_testing/collision_checking_options.h>
#include <pips/collision_testing/collision_checking_result.h>
#include <pips/utils/image_comparison_result.h>

//#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_eigen/tf2_eigen.h>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"






//#include "opencv2/highgui/highgui.hpp"

#include <sstream>
#include <limits> //for getting 'NAN'
#include <map>
#include <unordered_map>

// Only needed for dense model. Really, each class should have its own header
//#include <tf2_ros/transform_listener.h>
//#include <mesh_filter/depth_model.h>


// TODO: Move some of the generally useful functions (conversion/transformation/toString/etc) elsewhere

  inline
  void convertImage(const cv_bridge::CvImage::ConstPtr& cv_image_ref, cv::Mat& image)
  {
    image = cv_image_ref->image;
  }

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
// do opencv 2 code
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/core/ocl.hpp>
  inline
  void convertImage(const cv_bridge::CvImage::ConstPtr& cv_image_ref, cv::UMat& image)
  {
    image = cv_image_ref->image.getUMat(cv::ACCESS_READ);
  }
#endif








class HallucinatedRobotModelBase
{
  public:
    
    HallucinatedRobotModelBase()
    {
    }
 

    virtual CCResult testCollision(const geometry_msgs::Pose pose, CCOptions options)=0;
    virtual cv::Mat generateHallucinatedRobot(const geometry_msgs::Pose pose)=0;
    virtual void setParameters(double robot_radius, double robot_height, double floor_tolerance, double safety_expansion, bool show_im)=0;
    virtual bool inFrame(const cv::Point3d& pt){ return true;}

    
    void updateModel(const cv_bridge::CvImage::ConstPtr& cv_image_ref, double scale)
    {
      scale_ = scale;
      
      cv_image_ref_ = cv_image_ref;
      
      image_ref_ = getImage(cv_image_ref);
      
      doPrecomputation(cv_image_ref);
      
      /*
      if(show_im_)
      {
        double min;
        double max;
        cv::minMaxIdx(image_ref_, &min, &max);
        cv::Mat adjIm;
        cv::convertScaleAbs(image_ref_, adjIm, 255 / max);
        
        cv::imshow("Original image", adjIm);
        cv::waitKey(1);
      }
      */
    }
    

    void init(std::shared_ptr<image_geometry::PinholeCameraModel>& cam_model, const geometry_msgs::TransformStamped& base_optical_transform)
    {
      cam_model_ = cam_model;
      base_optical_transform_ = base_optical_transform;
    }
    
    void setTransform(const geometry_msgs::TransformStamped& base_optical_transform)
    {
      base_optical_transform_ = base_optical_transform;
    }
    
  protected:
    
    cv::Mat getImage(const cv_bridge::CvImage::ConstPtr& cv_image_ref)
    {
      cv::Mat image;
      convertImage(cv_image_ref, image);  // This was intended to be used in a templated class so that either UMat or Mat could be generated. In the future, my other classes will likely also be templated and the conversion will happen sooner
      return getImageImpl(image);
    }
    
    virtual cv::Mat getImageImpl(const cv::Mat& image)
    {
      return image;
    }

    virtual void doPrecomputation(const cv_bridge::CvImage::ConstPtr& cv_image_ref) {}
    
    virtual cv::Rect getImageRect()
    {
      return cv::Rect(cv::Point(0,0), image_ref_.size());
    }
   
  protected:
    std::shared_ptr<image_geometry::PinholeCameraModel> cam_model_;
    cv_bridge::CvImage::ConstPtr cv_image_ref_; //Allows access to original data and msg info
    double robot_radius_, robot_height_, floor_tolerance_;
    double scale_;
    bool show_im_=false;
    std::string name_ = "Undefined";
    geometry_msgs::TransformStamped base_optical_transform_;
    cv::Mat image_ref_; //Allows method to make local changes if needed


};



template<typename S> 
class HallucinatedRobotModelImpl : public HallucinatedRobotModelBase
{

  public: 
    
    HallucinatedRobotModelImpl() : HallucinatedRobotModelBase()
    {

    }
    
    virtual CCResult testCollision(const geometry_msgs::Pose pose, CCOptions options)
    {
      ROS_DEBUG_STREAM_NAMED(name_, "Collision request for model " << name_ << ": " << toString(pose));
      geometry_msgs::Pose pose_t = transformPose(pose);
      S convertedPose;
      convertPose(pose_t, convertedPose);
      ComparisonResult result = testCollisionImpl(convertedPose, options);
      
      if(options.get_details_ && result.collides() && result.hasDetails())
      {	
        std::vector<cv::Point3d> world_points;
        for(auto point : result.points())
        {
          ROS_DEBUG_STREAM_NAMED(name_, "Collision pixel coordinates: (" << point.pt.x << "," << point.pt.y << ")");
          cv::Point3d ray = cam_model_->projectPixelTo3dRay(point.pt);
          cv::Point3d worldPoint = ray * point.depth / scale_;
          world_points.push_back(worldPoint);
        }
	
	return CCResult(world_points);
      }
      
      return CCResult(result.collides());
      
    }
         
    virtual cv::Mat generateHallucinatedRobot(const geometry_msgs::Pose pose)
    {
      geometry_msgs::Pose pose_t = transformPose(pose);
      S convertedPose;
      convertPose(pose_t, convertedPose);
      return generateHallucinatedRobotImpl(convertedPose);
    }
    
    virtual cv::Rect getROI(const geometry_msgs::Pose pose)
    {
      geometry_msgs::Pose pose_t = transformPose(pose);
      S convertedPose;
      convertPose(pose_t, convertedPose);
      return getROIImpl(convertedPose);
    }
    
    
  protected:
  
    virtual ComparisonResult testCollisionImpl(const S pose, CCOptions options)=0;
    
    virtual cv::Mat generateHallucinatedRobotImpl(const S pose)
    {
      cv::Mat viz;
      if(std::numeric_limits<float>::has_quiet_NaN)
      {
        double dNaN = std::numeric_limits<float>::quiet_NaN();
        viz = cv::Mat(image_ref_.rows, image_ref_.cols, image_ref_.type(), cv::Scalar(dNaN));
      }
      else
      {
        viz = cv::Mat::zeros(image_ref_.rows, image_ref_.cols, image_ref_.type());
      }
      return viz;
    }
    
    virtual cv::Rect getROIImpl(const S pose)
    {
      cv::Rect imageBounds(0,0,cv_image_ref_->image.cols,cv_image_ref_->image.rows);
      return imageBounds;
    }
    
    virtual geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose)
    {
    // It would probably be better not to convert from whatever to geometry_msgs::Pose to Eigen::Affine3d back to geometry and then to whatever... But this seems the cleanest
      Eigen::Affine3d pose_eig;
      tf2::fromMsg(pose, pose_eig);

      //Transform coordinates of robot base into camera's optical frame
      Eigen::Affine3d pose_eig_t;
      
      tf2::doTransform(pose_eig, pose_eig_t, base_optical_transform_);
      
      geometry_msgs::Pose pose_t;
      convertPose(pose_eig_t, pose_t);
      
      ROS_DEBUG_STREAM_NAMED(name_, "Pose " << toString(pose) << " transformed to " << toString(pose_t) );
      
      return pose_t;
    }
};


namespace std
{
    template<> struct less<geometry_msgs::Pose>
    {
      double EPSILON = .01;
      bool is_equal(const double a, const double b) const
      {
        return fabs(a-b) < EPSILON;
      }
      
      bool is_less(const double a, const double b) const
      {
        return b-a > EPSILON;
      }
      
      bool operator() (const geometry_msgs::Pose& lhs, const geometry_msgs::Pose& rhs) const
       {
          return (
            (is_less(lhs.position.x,rhs.position.x)) ||
            ( is_equal(lhs.position.x,rhs.position.x) && is_less(lhs.position.y, rhs.position.y) ) ||
            ( is_equal(lhs.position.x,rhs.position.x) && is_equal(lhs.position.y, rhs.position.y) && is_less(lhs.position.z, rhs.position.z) ) 
          );
       }
      
      /*
       bool operator() (const geometry_msgs::Pose& lhs, const geometry_msgs::Pose& rhs) const
       {
          return (
            (lhs.position.x < rhs.position.x) ||
            ( (lhs.position.x == rhs.position.x) && (lhs.position.y < rhs.position.y) ) ||
            ( (lhs.position.x == rhs.position.x) && (lhs.position.y == rhs.position.y) && (lhs.position.z < rhs.position.z) ) 
          );
       }
       */
    };
}

template<typename T> 
class HallucinatedRobotModelCacheWrapper : public T
{
 
  private:
    
      struct CacheEntry
      {
        cv::Rect roi;
        cv::Mat image;
      };
      
      size_t cache_hits_=0;
      size_t cache_misses_=0;

  public: 
    
    HallucinatedRobotModelCacheWrapper() : T()
    {
      //Could probably prepend/append something to the name here?
    }
    
    virtual bool testCollision(const geometry_msgs::Pose pose, CCOptions options)
    {
      //ROS_DEBUG_STREAM_NAMED(name_, "Collision request for model " << name_ << ": " << toString(pose));
      
      CacheEntry results = generateCacheResults(pose);

      cv::Mat senseROI = cv::Mat(T::cv_image_ref_->image, results.roi);
      
      return isLessThan(senseROI,results.image);
    }
    
    
    virtual cv::Mat generateHallucinatedRobot(const geometry_msgs::Pose pose)
    {
      CacheEntry results = generateCacheResults(pose);
      //cv::Mat image = HallucinatedRobotModelImpl<S>::generateHallucinatedRobotImpl(pose);
      //image(results.roi) = results.image;
      return results.image;
    }
    

    virtual CacheEntry generateCacheResults(const geometry_msgs::Pose pose)
    {
      CacheEntry result;
      auto search = cache.find(pose);
      if(search != cache.end())
      {
        ++cache_hits_;
        result = search->second;
      }
      else
      {
        ++cache_misses_;
        cv::Mat fullImage = T::generateHallucinatedRobot(pose);
        cv::Rect roiRect = T::getROI(pose);
        cv::Mat image = cv::Mat(fullImage, roiRect);
        CacheEntry entry;
        entry.image = image.clone();
        entry.roi = roiRect;
        cache.insert(std::make_pair(pose,entry));
        result = entry;
      }
      ROS_INFO_STREAM_THROTTLE(1,"cache hits: " << cache_hits_ << ", cache misses: " << cache_misses_);

      return result;
    }
    
      
  private:
    
      std::map<geometry_msgs::Pose, CacheEntry> cache;

      
      //TODO: replace this with optimized version
    bool isLessThan(const cv::Mat& image1, const cv::Mat& image2)
    {
      cv::Mat res;
      cv::compare(image1, image2, res, cv::CMP_LT);

      int num_collisions = cv::countNonZero(res);
      bool collided = (num_collisions > 0);
      return collided;
    }
};


/*
class RectangularModelMinV : public RectangularModel
{
  protected:  
    double min_dist_ = -1;
    
  protected:
    void doPrecomputation()
    {
      cv::minMaxLoc(image_ref_, &min_dist_);  // This isn't enough: the floor is detected. Has to be the minimum value above plane.
      ROS_INFO_STREAM_NAMED(name_, "New minimum = " << min_dist_);
    }
  
    bool testCollisionImpl(const cv::Point3d pt)
    {
      if(pt.x + robot_radius_ < min_dist_ * scale_)
      {
        ROS_DEBUG_NAMED(name_, "Point in known free space");
        return false;
      }
      else
      {
        return RectangularModel::testCollisionImpl(pt);
      }

    }
    
    std::string getName() { return "RectangularModelMinV"; }
    
};

*/

/*
class DenseModel : public HallucinatedRobotModelImpl<geometry_msgs::Pose>
{
  public:
    DenseModel(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~DenseModel();
    

    void setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im);
    
  protected:
    bool testCollisionImpl(const geometry_msgs::Pose pose);
    cv::Mat generateHallucinatedRobotImpl(const geometry_msgs::Pose pose);
    std::string getName() { return "DenseModel"; }
    geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose);
    
  private:
    bool isReady();
    
  private:
    std::shared_ptr<depth_projection::DepthModel> depth_model_;
    
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    ros::NodeHandle nh_, pnh_;
    std::shared_ptr<image_transport::ImageTransport> model_depth_transport_;
    image_transport::CameraPublisher pub_model_depth_image_;
    
    bool isLessThan(const cv::Mat& depth_im, const cv::Mat& generated_im);
};
*/



#endif /*HALLUCINATED_ROBOT_MODEL_H*/

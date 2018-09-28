#ifndef PIPS_ROBOT_MODELS_COMPOSITE_MODEL_H
#define PIPS_ROBOT_MODELS_COMPOSITE_MODEL_H


#include <pips/collision_testing/robot_models/hallucinated_robot_model.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <pips/collision_testing/geometry_models/geometry_models.h>
#include <pips/collision_testing/geometry_models/cylinder.h>
#include <pips/collision_testing/geometry_models/box.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/MarkerArray.h>


namespace pips
{
  
  namespace collision_testing
  {
    
    namespace robot_models
    {

      class CompositeModel : public HallucinatedRobotModelImpl<geometry_msgs::Pose>
      {
      private:
        ros::NodeHandle nh_, pnh_;
        std::vector<std::shared_ptr<geometry_models::GeometryModel> > models_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        bool inited_ = false;
        bool transpose = true;
        ros::Publisher visualization_pub_;
        
        
      public:        
        CompositeModel(ros::NodeHandle nh, ros::NodeHandle pnh);

        
        void setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im);
      
      protected:
        cv::Mat generateHallucinatedRobotImpl(const geometry_msgs::Pose pose);
        
        ComparisonResult testCollisionImpl(const geometry_msgs::Pose pose, CCOptions options);
        
        virtual cv::Rect getColumnRect(const int x, const int y, const int width, const int height);
        
        cv::Rect getColumnRect(const cv::Rect& rect);
        
        virtual ComparisonResult isLessThan(const cv::Mat& col, float depth);
        
        virtual ComparisonResult isLessThanDetails(const cv::Mat& col, float depth);
        
        virtual geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose)
        {
          return pose;
        }
        
        visualization_msgs::MarkerArray::Ptr initMarkers(const std_msgs::Header& header, std::string ns);
        
        void addMarker(visualization_msgs::MarkerArray::Ptr& markers, const geometry_msgs::Pose& pose, const geometry_models::GeometryModel& model, const std_msgs::Header& header, std::string ns);
        
        
        cv::Mat getImageImpl(const cv::Mat& image)
        { 
          if(transpose)
          {
            cv::Mat transposed = image.t();
            return transposed;
          }
          return image;
        }
        
      private:
        bool init();
        
        // Header contains the frame to which transforms must be computed, subject to change
        bool updateTransforms(std_msgs::Header target_header);

        std::shared_ptr<geometry_models::GeometryModel> getGeometry(const std::string& tf_prefix, std::shared_ptr<pips::utils::AbstractCameraModel> cam_model_, const urdf::Link& link, const urdf::Collision& collision, unsigned int collision_ind);
        
        std::shared_ptr<geometry_models::GeometryModel> getGeometry(const urdf::Cylinder& cylinder);

        std::shared_ptr<geometry_models::GeometryModel> getGeometry(const urdf::Box& box);

      };
    }
  }
}

#endif //PIPS_ROBOT_MODELS_COMPOSITE_MODEL_H

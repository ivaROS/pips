#ifndef PIPS_ROBOT_MODEL_H
#define PIPS_ROBOT_MODEL_H

#include <pips/utils/pose_conversions.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <pips/RobotModelConfig.h>
#include <pips/collision_testing/geometry_models/generic_models.h>
#include <pips/collision_testing/geometry_models/cylinder.h>
#include <pips/collision_testing/geometry_models/box.h>
#include <tf2_utils/transform_manager.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <pips/utils/param_utils.h>

#include <string> //needed for converting numbers to string

namespace pips
{
  
  namespace collision_testing
  {
    
    namespace robot_models
    {

      class RobotModel
      {
      private:
        std::string name_;
        ros::NodeHandle nh_, pnh_;
        std::vector<std::shared_ptr<geometry_models::GenericGeometryModel> > models_;
        tf2_utils::TransformManager tfm_;
        bool inited_ = false;
        bool transpose = true;
        ros::Publisher visualization_pub_;
        geometry_msgs::TransformStamped base_optical_transform_;
        typedef dynamic_reconfigure::Server<pips::RobotModelConfig> ReconfigureServer;
        std::shared_ptr<ReconfigureServer> reconfigure_server_;
        std::string base_frame_id_;
        
        
      public:
        RobotModel(ros::NodeHandle nh, ros::NodeHandle pnh, tf2_utils::TransformManager tfm=tf2_utils::TransformManager(false));
        
        bool init();
        
        void setTransform(const geometry_msgs::TransformStamped transform);
        
        template <typename T, typename R=typename T::geometry_type>
        std::vector<R> getModel(const geometry_msgs::Pose pose)
        {
          
          const std_msgs::Header header = base_optical_transform_.header;
          
          std::vector<R> updated_models;
          
          geometry_msgs::TransformStamped origin_trans;
          origin_trans.transform.rotation = pose.orientation;
          origin_trans.transform.translation.x = pose.position.x;
          origin_trans.transform.translation.y = pose.position.y;
          origin_trans.transform.translation.z = pose.position.z;
          
          geometry_msgs::PoseStamped pose_stamped;
          pose_stamped.pose = pose;
          
          ROS_DEBUG_STREAM_NAMED(name_,"Pose: " << toString(pose));
          
          visualization_msgs::MarkerArray::Ptr markers;
          
          if(visualization_pub_.getNumSubscribers() > 0)
          {
            markers = initMarkers(header, "tested");
          }
          
          for(std::shared_ptr<geometry_models::GenericGeometryModel> model : models_)
          {
            
            //TODO: Switch to native tf2::Transform type
            geometry_msgs::TransformStamped model_pose_stamped;
            
            tf2::doTransform(model->current_transform_, model_pose_stamped, origin_trans);            
            
            ROS_DEBUG_STREAM_NAMED(name_,"Pose of [" << model->frame_id_ << "] in robot base frame [" << base_optical_transform_.child_frame_id << "]: " << toString(model_pose_stamped));
            
            geometry_msgs::TransformStamped camera_pose_stamped;
            
            tf2::doTransform(model_pose_stamped, camera_pose_stamped, base_optical_transform_);
            
            
            ROS_DEBUG_STREAM_NAMED(name_,"Pose of [" << model->frame_id_ << "] in current sensor frame [" << header.frame_id << "]: " << toString(camera_pose_stamped));
            
            geometry_msgs::Pose model_pose;
            model_pose.position.x = camera_pose_stamped.transform.translation.x;
            model_pose.position.y = camera_pose_stamped.transform.translation.y;
            model_pose.position.z = camera_pose_stamped.transform.translation.z;
            model_pose.orientation = camera_pose_stamped.transform.rotation;
            
            if(markers)
            {
              addMarker(markers, model_pose, *model, header, "tested");
            }
            
            
            //NOTE: Currently, only the position (not orientation) is transformed
            model_pose.orientation = pose.orientation; //camera_pose_stamped.transform.rotation;
            
            auto converted_model = T::convert(model, model_pose);
            if(converted_model)
            {
              updated_models.push_back(converted_model);
            }
          }
          
          if(markers)
            visualization_pub_.publish(markers);
          
          
          return updated_models;
        }
        
      private:
        void reconfigureCB(const pips::RobotModelConfig& config, uint32_t level);
        
        bool updateTransforms();

        std::shared_ptr<geometry_models::GenericGeometryModel> getGeometry(const std::string& tf_prefix, const urdf::Link& link, const urdf::Collision& collision, unsigned int collision_ind);
        
        std::shared_ptr<geometry_models::GenericGeometryModel> getGeometry(const urdf::Cylinder& cylinder);
        
        std::shared_ptr<geometry_models::GenericGeometryModel> getGeometry(const urdf::Box& box);
        
        visualization_msgs::MarkerArray::Ptr initMarkers(const std_msgs::Header& header, std::string ns);

        void addMarker(visualization_msgs::MarkerArray::Ptr& markers, const geometry_msgs::Pose& pose, const geometry_models::GenericGeometryModel& model, const std_msgs::Header& header, std::string ns);

      };        

    }
    
  }
  
}

#endif //PIPS_ROBOT_MODEL_H

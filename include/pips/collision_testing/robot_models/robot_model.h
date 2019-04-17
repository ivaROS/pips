#ifndef PIPS_ROBOT_MODEL_H
#define PIPS_ROBOT_MODEL_H

#include <pips/utils/pose_conversions.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <pips/collision_testing/geometry_models/geometry_models.h>
#include <pips/collision_testing/geometry_models/cylinder.h>
#include <pips/collision_testing/geometry_models/box.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


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
        std::vector<std::shared_ptr<geometry_models::GeometryModel> > models_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        bool inited_ = false;
        bool transpose = true;
        ros::Publisher visualization_pub_;
        geometry_msgs::TransformStamped base_optical_transform_;
        
      public:
        
        RobotModel(ros::NodeHandle nh, ros::NodeHandle pnh):
          //RobotModelImpl<geometry_msgs::Pose>(),
          name_("robot_model"),
          nh_(nh),
          pnh_(pnh, name_),
          //tf_buffer_(),
          tf_listener_(tf_buffer_)
        {
        }
        
        //TODO: add dynamic reconfiguration
        bool init()
        {          
          if(inited_)
            return true;
          
          inited_=true;
          std::string xml_string;
          std::string tf_prefix="";

          
          std::string param_name = "/robot_description";
          if( !pnh_.getParam("param_name", param_name) )
          {
            pnh_.setParam("param_name", param_name);
            ROS_WARN_STREAM_NAMED(name_,"No robot description parameter name provided. Using default '" << param_name << "'");
            //return false;
          }
          
          if( !nh_.getParam(param_name, xml_string) )
          {
            ROS_ERROR_NAMED(name_,"No robot description found!");
            return false;
          }
          
          if( pnh_.getParam("tf_prefix", tf_prefix) )
          {
            ROS_INFO_STREAM("tf_prefix =\"" << tf_prefix << "\"");
            //return false;
            tf_prefix = tf_prefix + "/";
          }
          else
          {
            tf_prefix = "";
          }
          
          //https://github.com/ros/urdfdom/blob/06f5f9bc34f09b530d9f3743cb0516934625da54/urdf_parser/src/model.cpp#L62
          urdf::ModelInterfaceConstSharedPtr model = urdf::parseURDF(xml_string);
          
          //Iterate over links and add collision geometry
          //https://github.com/ros/urdfdom/blob/06f5f9bc34f09b530d9f3743cb0516934625da54/urdf_parser/src/model.cpp#L263
          for (std::map<std::string, urdf::LinkSharedPtr>::const_iterator l=model->links_.begin(); l!=model->links_.end(); l++)  
          {
            //https://github.com/ros-visualization/rviz/blob/87742e4b8f40d8fe7e2a6952065651266faf73ba/src/rviz/robot/robot.cpp#L273
            const urdf::LinkConstSharedPtr& link = l->second;
            const std::string link_name = l->first;
            
            unsigned int num_collisions = link->collision_array.size();
            unsigned int collision_ind = 0;
            //https://github.com/ros-visualization/rviz/blob/87742e4b8f40d8fe7e2a6952065651266faf73ba/src/rviz/robot/robot_link.cpp#L698
            for(std::vector<urdf::CollisionSharedPtr >::const_iterator vi = link->collision_array.begin(); vi != link->collision_array.end(); vi++, collision_ind++ )
            {
              urdf::CollisionConstSharedPtr collision = *vi;
              if( collision && collision->geometry )
              {
                std::shared_ptr<geometry_models::GeometryModel> model = getGeometry(tf_prefix, *link, *collision, collision_ind);
                
                if(model)
                {
                  models_.push_back(model);
                }
              }
              
            }

            

          }

          ros::Duration d(.1);
          while(!updateTransforms())
          {
            d.sleep();
          }
          
          visualization_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("markers",5);
          
          
          return true;
          //TODO: Precompute all relevant transforms between links and camera
        }

//         void doPrecomputation(const cv_bridge::CvImage::ConstPtr& cv_image_ref)
//         {
//           
//         }
        
        
        
        // Header contains the frame to which transforms must be computed, subject to change
        bool updateTransforms()
        {
          const std::string& target_frame_id = base_optical_transform_.header.frame_id;
          const std::string& base_frame_id = base_optical_transform_.child_frame_id;
          
          geometry_msgs::TransformStamped base_camera_rotation = base_optical_transform_;
          geometry_msgs::Vector3 empty_translation;
          base_camera_rotation.transform.translation = empty_translation;
          
          bool all_good = true;
          for(std::shared_ptr<geometry_models::GeometryModel> model : models_)
          {
            const std::string& model_frame_id = model->frame_id_;
            const std::string& model_name = model->name_;
            const geometry_msgs::TransformStamped& origin_link_transform = model->origin_transform_;
            geometry_msgs::TransformStamped& current_transform = model->current_transform_;
            
            
            try
            {
              geometry_msgs::TransformStamped origin_base_transform = tf_buffer_.lookupTransform ( base_frame_id, model_frame_id, ros::Time(0));
              
              current_transform = origin_base_transform;
              
              ROS_INFO_STREAM_NAMED(name_,"Origin:Base = (" << model_frame_id << ":" << base_frame_id << ")= " << toString(origin_base_transform));
              
              
            } catch ( tf2::TransformException &ex ) {
              ROS_WARN_STREAM ("Problem finding transform:\n" <<ex.what() );
              all_good = false;
            }
            
          }
          return all_good;
        }
        
        void setTransform(const geometry_msgs::TransformStamped transform)
        {
          base_optical_transform_ = transform;
        }

        template <typename R, typename T>
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
          
          for(std::shared_ptr<geometry_models::GeometryModel> model : models_)
          {

            
            geometry_msgs::TransformStamped model_pose_stamped;
            
            tf2::doTransform(model->current_transform_, model_pose_stamped, origin_trans);            
            
            ROS_DEBUG_STREAM_NAMED(name_,"Pose of [" << model->frame_id_ << "] in Base frame: " << toString(model_pose_stamped));
            
            geometry_msgs::TransformStamped camera_pose_stamped;
            
            tf2::doTransform(model_pose_stamped, camera_pose_stamped, base_optical_transform_);
            
            
            ROS_DEBUG_STREAM_NAMED(name_,"Pose of [" << model->frame_id_ << "] in Camera frame: " << toString(camera_pose_stamped));

            geometry_msgs::Pose model_pose;
            model_pose.position.x = camera_pose_stamped.transform.translation.x;
            model_pose.position.y = camera_pose_stamped.transform.translation.y;
            model_pose.position.z = camera_pose_stamped.transform.translation.z;
            model_pose.orientation = camera_pose_stamped.transform.rotation;
            
            //TODO: only create markers if subscriber exists
            addMarker(markers, model_pose, *model, header, "tested");
            
            
            //NOTE: Currently, only the position (not orientation) is transformed
            model_pose.orientation = pose.orientation; //camera_pose_stamped.transform.rotation;
            
            R converted_model = T::convert(*model, model_pose);
            if(converted_model)
            {
              updated_models.push_back(converted_model);
            }
          }
          
          if(markers)
            visualization_pub_.publish(markers);
          
          
          return updated_models;
        }
          
        
        
        std::shared_ptr<geometry_models::GeometryModel> getGeometry(const std::string& tf_prefix, const urdf::Link& link, const urdf::Collision& collision, unsigned int collision_ind)
        {
          const std::string& link_name = link.name;
          
          const urdf::GeometrySharedPtr& geom = collision.geometry;
          const urdf::Pose& origin = collision.origin;
          std::string name = collision.name;
          
          if(name == "")
          {
            name = std::to_string(collision_ind);
          }
          
          const std::string link_frame_id = tf_prefix + link_name;
          
          
          
          
          
          std::shared_ptr<geometry_models::GeometryModel> model;
          
          // https://github.com/ros/urdfdom/blob/06f5f9bc34f09b530d9f3743cb0516934625da54/urdf_parser/src/link.cpp#L547
          if (urdf::dynamic_pointer_cast<urdf::Sphere>(geom))
          {
            //addSphere((*(urdf::dynamic_pointer_cast<Sphere>(geom).get())));
          }
          else if (urdf::dynamic_pointer_cast<urdf::Box>(geom))
          {
            model = getGeometry((*(urdf::dynamic_pointer_cast<urdf::Box>(geom).get())));
            //ROS_ERROR_STREAM("Box type not currently correct, ignoring!");
          }
          else if (urdf::dynamic_pointer_cast<urdf::Cylinder>(geom))
          {
            model = getGeometry((*(urdf::dynamic_pointer_cast<urdf::Cylinder>(geom).get())));
          }
          else if (urdf::dynamic_pointer_cast<urdf::Mesh>(geom))
          {
            ROS_ERROR_STREAM("Mesh type not supported!");
          }
          else
          {
            ROS_ERROR_STREAM("Unknown type not supported!");
          }
          
          if(model)
          {
            //TODO: Move a lot of this somewhere else, possibly into geometry_models or some factory class
            
            //Assign things that always need to be assigned
            model->name_ = name;            
            
//             const urdf::Pose tmp;
//             if(origin == tmp) //no offset, so just use link's frame
//             {
//               model->frame_id_ = link_frame_id;
//             }
//             else
            {
              model->origin_ = origin;
              model->frame_id_ = link_frame_id + "/" + name;
              
              geometry_msgs::TransformStamped origin_pose;
              
              geometry_msgs::Vector3& t = origin_pose.transform.translation;
              geometry_msgs::Quaternion& rot = origin_pose.transform.rotation;
              
              t.x=origin.position.x;
              t.y=origin.position.y;
              t.z=origin.position.z;
              
              origin.rotation.getQuaternion(rot.x,rot.y,rot.z,rot.w);
              
              tf2::Stamped<tf2::Transform> tf_origin_pose;
              tf2::fromMsg(origin_pose, tf_origin_pose);
              
              tf2::Transform tf_origin_transform = tf_origin_pose.inverse();
              
              geometry_msgs::TransformStamped transform = origin_pose;
              //transform.transform = tf2::toMsg(tf_origin_transform);
              
              transform.child_frame_id = model->frame_id_;
              transform.header.frame_id = link_frame_id;
              
              model->origin_transform_ = transform;

              tf_buffer_.setTransform(transform, "urdf", true); //Add static transform for origin of collision model
            }
          }
          
          return model;
        }
        
        std::shared_ptr<geometry_models::GeometryModel> getGeometry(const urdf::Cylinder& cylinder)
        {
          ROS_INFO_STREAM("Adding Cylinder primitive. Radius=" << cylinder.radius << ", length=" << cylinder.length);
          std::shared_ptr<geometry_models::Cylinder> model = std::make_shared<geometry_models::Cylinder>(cylinder.radius, cylinder.length);
          return model;
        }
        
        std::shared_ptr<geometry_models::GeometryModel> getGeometry(const urdf::Box& box)
        {
          ROS_INFO_STREAM("Adding Box primitive. Length=" << box.dim.x << ", Width=" << box.dim.y << ", Height=" << box.dim.z);
          std::shared_ptr<geometry_models::Box> model = std::make_shared<geometry_models::Box>(box.dim.x, box.dim.y, box.dim.z);
          return model;
        }
        
        
        
        
        visualization_msgs::MarkerArray::Ptr initMarkers(const std_msgs::Header& header, std::string ns)
        {
          visualization_msgs::MarkerArray::Ptr markers = boost::make_shared<visualization_msgs::MarkerArray>();
        
          visualization_msgs::Marker marker;
          marker.ns = ns;
          marker.header = header;
          marker.action = visualization_msgs::Marker::DELETEALL;
          
          markers->markers.push_back(marker);
          return markers;
          
          //        visualization_pub_.publish(markers);
          
        }

        void addMarker(visualization_msgs::MarkerArray::Ptr& markers, const geometry_msgs::Pose& pose, const geometry_models::GeometryModel& model, const std_msgs::Header& header, std::string ns)
        {
          if(markers)
          {
            visualization_msgs::Marker marker = model.marker_;
            
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.a = .25;
            marker.color.r = 1;
            marker.ns = ns;
            
            marker.pose=pose;
            marker.header = header;
            
            marker.id = markers->markers.size();
          
            markers->markers.push_back(marker);
          }
        }

      };        

    }
    
  }
  
}

#endif //PIPS_ROBOT_MODEL_H

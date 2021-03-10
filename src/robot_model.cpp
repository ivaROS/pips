
#include <pips/collision_testing/robot_models/robot_model.h>

#include <pips/utils/pose_conversions.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <pips/RobotModelConfig.h>
#include <pips/collision_testing/geometry_models/generic_models.h>
#include <pips/collision_testing/geometry_models/cylinder.h>
#include <pips/collision_testing/geometry_models/box.h>
#include <pips/collision_testing/geometry_models/sphere.h>
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
     
      //Function copied from https://thispointer.com/c-how-to-check-if-a-string-ends-with-an-another-given-string/
      /*
       * Case Insensitive Implementation of endsWith()
       * It checks if the string 'mainStr' ends with given string 'toMatch'
       */
      bool endsWithCaseInsensitive(std::string mainStr, std::string toMatch)
      {
        auto it = toMatch.begin();
        return mainStr.size() >= toMatch.size() &&
        std::all_of(std::next(mainStr.begin(),mainStr.size() - toMatch.size()), mainStr.end(), [&it](const char & c){
          return ::tolower(c) == ::tolower(*(it++))  ;
        } );
      }

      RobotModel::RobotModel(ros::NodeHandle nh, ros::NodeHandle pnh, tf2_utils::TransformManager tfm):
        name_("robot_model"),
        nh_(nh),
        pnh_(pnh, name_),
        tfm_(tfm, nh)
      {
        reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
      }
        
      void RobotModel::reconfigureCB(const pips::RobotModelConfig& config, uint32_t level)
      {          
        ROS_INFO_STREAM("Reconfigure callback: param_name: " << config.param_name << ", inflation: " << config.safety_expansion << ", floor tolerance: " << config.floor_tolerance);
        std::string xml_string;
        std::string tf_prefix="";

        
        if( !nh_.getParam(config.param_name, xml_string) )
        {
          ROS_ERROR_STREAM_NAMED(name_,"No robot description found at [" << config.param_name << "]");
          return;
        }
        
        if( pnh_.getParam("tf_prefix", tf_prefix) )
        {
          ROS_INFO_STREAM("tf_prefix =\"" << tf_prefix << "\"");
          tf_prefix = tf_prefix + "/";
        }
        else
        {
          tf_prefix = "";
        }
        
        ROS_INFO_STREAM("Clearing the [" << models_.size() << "] old geometry primitives...");
        models_.clear();
        
        //https://github.com/ros/urdfdom/blob/06f5f9bc34f09b530d9f3743cb0516934625da54/urdf_parser/src/model.cpp#L62
        urdf::ModelInterfaceConstSharedPtr urdf_model = urdf::parseURDF(xml_string);
        
        //Iterate over links and add collision geometry
        //https://github.com/ros/urdfdom/blob/06f5f9bc34f09b530d9f3743cb0516934625da54/urdf_parser/src/model.cpp#L263
        for (std::map<std::string, urdf::LinkSharedPtr>::const_iterator l=urdf_model->links_.begin(); l!=urdf_model->links_.end(); l++)  
        {
          //https://github.com/ros-visualization/rviz/blob/87742e4b8f40d8fe7e2a6952065651266faf73ba/src/rviz/robot/robot.cpp#L273
          const urdf::LinkConstSharedPtr& link = l->second;
          const std::string link_name = l->first;
          
          unsigned int collision_ind = 0;
          //https://github.com/ros-visualization/rviz/blob/87742e4b8f40d8fe7e2a6952065651266faf73ba/src/rviz/robot/robot_link.cpp#L698
          for(std::vector<urdf::CollisionSharedPtr >::const_iterator vi = link->collision_array.begin(); vi != link->collision_array.end(); vi++, collision_ind++ )
          {
            urdf::CollisionConstSharedPtr collision = *vi;
            if( collision && collision->geometry )
            {
              std::shared_ptr<geometry_models::GenericGeometryModel> geometry_model = getGeometry(tf_prefix, *link, *collision, collision_ind);
              
              if(geometry_model)
              {
                models_.push_back(geometry_model);
              }
            }
            
          }

        }

        ros::Duration d(.1);
        while(!updateTransforms())
        {
          d.sleep();
        }
        
        for(std::shared_ptr<geometry_models::GenericGeometryModel> model : models_)
        {
          //model->adjust(config.safety_expansion, config.floor_tolerance);
        }
      }
        
      bool RobotModel::init()
      {
        if(!inited_)
        { 
          pips::utils::searchParam(pnh_, "base_frame_id", base_frame_id_, "base_link");
          ROS_INFO_STREAM_NAMED("init", "[RobotModel]: base_frame_id: " << base_frame_id_);
          
          reconfigure_server_->setCallback(boost::bind(&RobotModel::reconfigureCB, this, _1, _2));
          visualization_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("markers",5);  
          
          inited_=true;
        }
        return true;
      }

      bool RobotModel::updateTransforms()
      {
        bool all_good = true;
        for(std::shared_ptr<geometry_models::GenericGeometryModel> model : models_)
        {
          const std::string& model_frame_id = model->frame_id_;
          //const std::string& model_name = model->name_;
          //const geometry_msgs::TransformStamped& origin_link_transform = model->origin_transform_;
          geometry_msgs::TransformStamped& current_transform = model->current_transform_;
          
          
          try
          {
            geometry_msgs::TransformStamped origin_base_transform = tfm_.getBuffer()->lookupTransform ( base_frame_id_, model_frame_id, ros::Time(0));
            
            current_transform = origin_base_transform;
            
            ROS_INFO_STREAM_NAMED("updateTransforms","Origin:Base = (" << model_frame_id << ":" << base_frame_id_ << ")= " << toString(origin_base_transform));
            
            
          } catch ( tf2::TransformException &ex ) {
            ROS_WARN_STREAM_NAMED("updateTransforms","Problem finding transform:\n" <<ex.what() );
            all_good = false;
          }
          
        }
        return all_good;
      }
        
     
      void RobotModel::setTransform(const geometry_msgs::TransformStamped transform)
      {
        base_optical_transform_ = transform;
      }
      
      std::shared_ptr<geometry_models::GenericGeometryModel> RobotModel::getGeometry(const std::string& tf_prefix, const urdf::Link& link, const urdf::Collision& collision, unsigned int collision_ind)
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

        std::shared_ptr<geometry_models::GenericGeometryModel> model;
        
        // https://github.com/ros/urdfdom/blob/06f5f9bc34f09b530d9f3743cb0516934625da54/urdf_parser/src/link.cpp#L547
        if (urdf::dynamic_pointer_cast<urdf::Sphere>(geom))
        {
          model = getGeometry((*(urdf::dynamic_pointer_cast<urdf::Sphere>(geom).get())));
        }
        else if (urdf::dynamic_pointer_cast<urdf::Box>(geom))
        {
          if(endsWithCaseInsensitive(name, "ellipsoid"))
          {
            model = getEllipsoid((*(urdf::dynamic_pointer_cast<urdf::Box>(geom).get())));
          }
          else
          {
            model = getGeometry((*(urdf::dynamic_pointer_cast<urdf::Box>(geom).get())));
          }
        }
        else if (urdf::dynamic_pointer_cast<urdf::Cylinder>(geom))
        {
          model = getGeometry((*(urdf::dynamic_pointer_cast<urdf::Cylinder>(geom).get())));
        }
        else if (urdf::dynamic_pointer_cast<urdf::Mesh>(geom))
        {
          ROS_WARN_STREAM("Mesh type not supported!");
        }
        else
        {
          ROS_WARN_STREAM("Unknown type not supported!");
        }
        
        if(model)
        {
          //TODO: Move a lot of this somewhere else, possibly into geometry_models or some factory class
          
          //Assign things that always need to be assigned
          model->name_ = name;            
          
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
                          
            geometry_msgs::TransformStamped transform = origin_pose;              
            transform.child_frame_id = model->frame_id_;
            transform.header.frame_id = link_frame_id;
            
            model->origin_transform_ = transform;

            tfm_.getBuffer()->setTransform(transform, "urdf", true); //Add static transform for origin of collision model
          }
        }
        
        return model;
      }
      
      std::shared_ptr<geometry_models::GenericGeometryModel> RobotModel::getGeometry(const urdf::Cylinder& cylinder)
      {
        ROS_INFO_STREAM("Adding Cylinder primitive. Radius=" << cylinder.radius << ", length=" << cylinder.length);
        std::shared_ptr<geometry_models::Cylinder> model = std::make_shared<geometry_models::Cylinder>(cylinder.radius, cylinder.length);
        return model;
      }
      
      std::shared_ptr<geometry_models::GenericGeometryModel> RobotModel::getGeometry(const urdf::Box& box)
      {
        ROS_INFO_STREAM("Adding Box primitive. Length=" << box.dim.x << ", Width=" << box.dim.y << ", Height=" << box.dim.z);
        std::shared_ptr<geometry_models::Box> model = std::make_shared<geometry_models::Box>(box.dim.x, box.dim.y, box.dim.z);
        return model;
      }
      
      std::shared_ptr<geometry_models::GenericGeometryModel> RobotModel::getGeometry(const urdf::Sphere& sphere)
      {
        ROS_INFO_STREAM("Adding Sphere primitive. Radius=" << sphere.radius);
        std::shared_ptr<geometry_models::Sphere> model = std::make_shared<geometry_models::Sphere>(sphere.radius);
        return model;
      }
      
      std::shared_ptr<geometry_models::GenericGeometryModel> RobotModel::getEllipsoid(const urdf::Box& box)
      {
        ROS_INFO_STREAM("Adding Ellipsoid primitive. Length=" << box.dim.x << ", Width=" << box.dim.y << ", Height=" << box.dim.z);
        std::shared_ptr<geometry_models::Ellipsoid> model = std::make_shared<geometry_models::Ellipsoid>(box.dim.x, box.dim.y, box.dim.z);
        return model;
      }
      
      visualization_msgs::MarkerArray::Ptr RobotModel::initMarkers(const std_msgs::Header& header, std::string ns)
      {
        visualization_msgs::MarkerArray::Ptr markers = boost::make_shared<visualization_msgs::MarkerArray>();
      
        visualization_msgs::Marker marker;
        marker.ns = ns;
        marker.header = header;
        marker.action = visualization_msgs::Marker::DELETEALL;
        
        markers->markers.push_back(marker);
        return markers;
      }

      void RobotModel::addMarker(visualization_msgs::MarkerArray::Ptr& markers, const geometry_msgs::Pose& pose, const geometry_models::GenericGeometryModel& model, const std_msgs::Header& header, std::string ns)
      {
        if(markers)
        {
          visualization_msgs::Marker marker = model.marker_;
          
          marker.action = visualization_msgs::Marker::ADD;
          marker.color.a = .25;
          marker.color.r = 1;
          marker.ns = ns;
          
          marker.scale.x = std::max(marker.scale.x, 0.01);
          marker.scale.y = std::max(marker.scale.y, 0.01);
          marker.scale.z = std::max(marker.scale.z, 0.01);
          
          marker.pose=pose;
          marker.header = header;
          
          marker.id = markers->markers.size();
        
          markers->markers.push_back(marker);
        }
      }      

    }
    
  }
  
}

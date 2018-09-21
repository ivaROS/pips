//   GeometrySharedPtr geom
// Sphere s
// s.type = Geometry::SPHERE;
// s.radius
// 
// Box b
// type = BOX
//     b.dim.init(c->Attribute("size"));
// dim.x,y,z
// 
// Cylinder y
//   y.type = Geometry::CYLINDER;
// y.length
// y.radius
// 
// 
// Collision col 
// col.geometry = geom
// 
// Link link
// link.collision=col
// link.collision_array
// link.name
// 
// model->links_: std__pair(link->name, link)
// 
// for (std::map<std::string, LinkSharedPtr>::const_iterator l=model.links_.begin(); l!=model.links_.end(); l++)  
// 
// 
//   
//   
//   
//   
//   
//   
//   
  
  
  
//#include <pips/collision_testing/robot_models/hallucinated_robot_model.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <pips/collision_testing/geometry_models/geometry_models.h>
#include <pips/collision_testing/geometry_models/cylinder.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace pips
{
  
  namespace collision_testing
  {
    
    namespace robot_models
    {

      class CompositeModel //: HallucinatedRobotModelImpl<geometry_msgs::Pose>
      {
      private:
        ros::NodeHandle nh_, pnh_;
        std::vector<std::shared_ptr<geometry_models::GeometryModel> > models_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        
        
      public:
        CompositeModel(ros::NodeHandle nh, ros::NodeHandle pnh):
          nh_(nh),
          pnh_(pnh),
          tf_listener_(tf_buffer_)
        {
        }
        
        
        bool init()
        {
          
          std::string xml_string;
          std::string tf_prefix="";
          //TODO: load description & tf_prefix from parameter server
//           if( !pnh_.getParam("robot_description", xml_string) )
//           {
//             ROS_ERROR("No robot description provided!");
//             return false;
//           }
          
          std::string param_name;
          if( !pnh_.getParam("param_name", param_name) )
          {
            ROS_ERROR("No robot description parameter name provided!");
            return false;
          }
          
          if( !nh_.getParam(param_name, xml_string) )
          {
            ROS_ERROR("No robot description found!");
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
            
            //https://github.com/ros-visualization/rviz/blob/87742e4b8f40d8fe7e2a6952065651266faf73ba/src/rviz/robot/robot_link.cpp#L698
            for(std::vector<urdf::CollisionSharedPtr >::const_iterator vi = link->collision_array.begin(); vi != link->collision_array.end(); vi++ )
            {
              urdf::CollisionConstSharedPtr collision = *vi;
              if( collision && collision->geometry )
              {
                std::shared_ptr<geometry_models::GeometryModel> model = getGeometry(*collision);
                
                //TODO: I may move all assignments to lower levels so I don't have to perform these checks
                if(model)
                {
                  model->frame_id_ = tf_prefix + link_name;
                  models_.push_back(model);
                }
              }
              
            }

            

          }
          
          std_msgs::Header header;
          header.frame_id = "camera_depth_optical_frame";
          
          ros::Duration d(.1);
          while(!updateTransforms(header))
          {
            d.sleep();
          }
            
          
          return true;
          //TODO: Precompute all relevant transforms between links and camera
        }

//         cv::Mat generateHallucinatedRobotImpl(const geometry_msgs::Pose& pose)
//         {
//           cv::Mat viz = HallucinatedRobotModelImpl::generateHallucinatedRobotImpl(pose);
//           
//           
//           for(std::shared_ptr<geometry_models::GeometryModel> model : models_)
//           {
//             geometry_msgs::Pose model_pose;
//             tf2::doTransform(pose, model_pose, model->current_transform_);
//             
//             
//           }
//           
//           return viz;
//         }
//         
//         
//         ComparisonResult CylindricalModel::testCollisionImpl(const cv::Point3d pt, CCOptions options)
//         {
//           std::vector<COLUMN_TYPE> cols = getColumns(pt);
//           
//           ComparisonResult result;
//           for(unsigned int i = 0; i < cols.size(); ++i)
//           {
//             //TODO: limit rect to image size
//             cv::Rect roi = getColumnRect(cols[i].rect);
//             cv::Mat col = cv::Mat(this->image_ref_,cols[i].rect); //cols[i].image;
//             float depth = cols[i].depth;
//             
//             ComparisonResult column_result = isLessThan(col, depth);
//             
//             if(column_result && options)
//             {
//               if(!column_result.hasDetails())
//               {
//                 column_result = isLessThanDetails(col,depth);
//               }
//               cv::Point offset;
//               cv::Size size;
//               col.locateROI(size, offset);
//               
//               column_result.addOffset(offset);	
//               
//               if(show_im_)
//               {
//                 result.addResult(column_result);
//               }
//               else
//               {
//                 return column_result;
//               }
//             }
//           }
//           
//           
//           
//           return result;
//         }
//         
        
        
        
      private:
        
        // Header contains the frame to which transforms must be computed, subject to change
        bool updateTransforms(std_msgs::Header target_header)
        {
          bool all_good = true;
          for(std::shared_ptr<geometry_models::GeometryModel> model : models_)
          {
            try
            {
              geometry_msgs::TransformStamped link_transform = tf_buffer_.lookupTransform ( target_header.frame_id, model->frame_id_, ros::Time(0));
              
              tf2::doTransform(model->origin_transform_, model->current_transform_, link_transform);
              
              ROS_INFO_STREAM("Link transform = (" << link_transform.transform.translation.x << ", " << link_transform.transform.translation.y << ", " << link_transform.transform.translation.z << ") [" << link_transform.transform.rotation.x << ", " << link_transform.transform.rotation.y << ", " << link_transform.transform.rotation.z << ", " << link_transform.transform.rotation.w << "]");
              
              ROS_INFO_STREAM("Origin transform = (" << model->origin_transform_.transform.translation.x << ", " << model->origin_transform_.transform.translation.y << ", " << model->origin_transform_.transform.translation.z << ") [" << model->origin_transform_.transform.rotation.x << ", " << model->origin_transform_.transform.rotation.y << ", " << model->origin_transform_.transform.rotation.z << ", " << model->origin_transform_.transform.rotation.w << "]");
              
              ROS_INFO_STREAM("Combined transform = (" << model->current_transform_.transform.translation.x << ", " << model->current_transform_.transform.translation.y << ", " << model->current_transform_.transform.translation.z << ") [" << model->current_transform_.transform.rotation.x << ", " << model->current_transform_.transform.rotation.y << ", " << model->current_transform_.transform.rotation.z << ", " << model->current_transform_.transform.rotation.w << "]");
              
            } catch ( tf2::TransformException &ex ) {
              ROS_WARN_STREAM ("Problem finding transform:\n" <<ex.what() );
              all_good = false;
            }
            
          }
          return all_good;
        }
        
        std::shared_ptr<geometry_models::GeometryModel> getGeometry(const urdf::Collision& collision)
        {
          const urdf::GeometrySharedPtr& geom = collision.geometry;
          const urdf::Pose& origin = collision.origin;
          const std::string& name = collision.name;
          
          std::shared_ptr<geometry_models::GeometryModel> model;
          
          // https://github.com/ros/urdfdom/blob/06f5f9bc34f09b530d9f3743cb0516934625da54/urdf_parser/src/link.cpp#L547
          if (urdf::dynamic_pointer_cast<urdf::Sphere>(geom))
          {
            //addSphere((*(urdf::dynamic_pointer_cast<Sphere>(geom).get())));
          }
          else if (urdf::dynamic_pointer_cast<urdf::Box>(geom))
          {
            //addBox((*(urdf::dynamic_pointer_cast<urdf::Box>(geom).get())));
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
            model->origin_ = origin;
            
            geometry_msgs::TransformStamped transform;
            
            geometry_msgs::Vector3& t = transform.transform.translation;
            geometry_msgs::Quaternion& rot = transform.transform.rotation;
            
            t.x=origin.position.x;
            t.y=origin.position.y;
            t.z=origin.position.z;
            
            origin.rotation.getQuaternion(rot.x,rot.y,rot.z,rot.w);
            
            model->origin_transform_ = transform;
          }
          
          return model;
        }
        
        std::shared_ptr<geometry_models::GeometryModel> getGeometry(const urdf::Cylinder& cylinder)
        {
          ROS_INFO_STREAM("Adding Cylinder primitive. Radius=" << cylinder.radius << ", length=" << cylinder.length);
          std::shared_ptr<geometry_models::Cylinder> model = std::make_shared<geometry_models::Cylinder>(cylinder.radius, cylinder.length);
          return model;
        }
        
        
        
        
        
      };

    }
    
  }
  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "composite_model");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pips::collision_testing::robot_models::CompositeModel model(nh,pnh);
  model.init();
  
  ros::spin();
  
}

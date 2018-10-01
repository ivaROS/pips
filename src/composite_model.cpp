#include <pips/collision_testing/robot_models/composite_model.h>

#include <pips/collision_testing/robot_models/hallucinated_robot_model.h>
#include <pips/utils/image_comparison_implementations.h>

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

      
        CompositeModel::CompositeModel(ros::NodeHandle nh, ros::NodeHandle pnh):
          HallucinatedRobotModelImpl<geometry_msgs::Pose>(),
          nh_(nh),
          pnh_(pnh),
          //tf_buffer_(),
          tf_listener_(tf_buffer_)
        {
          this->name_ = "CompositeModel";
        }
        
        //This is only called after the transform, scale, and cam_model_ have been assigned
        void CompositeModel::setParameters(double radius, double height, double safety_expansion, double floor_tolerance, bool show_im)
        {
          if(!inited_)
          {
            init();
            inited_ = true;
          }
          show_im_ = show_im;
        }
        
        
        bool CompositeModel::init()
        {          
          std::string xml_string;
          std::string tf_prefix="";

          
          std::string param_name = "/robot_description";
          if( !pnh_.getParam("param_name", param_name) )
          {
            pnh_.setParam("param_name", param_name);
            ROS_WARN_STREAM("No robot description parameter name provided. Using default '" << param_name << "'");
            //return false;
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
            
            unsigned int num_collisions = link->collision_array.size();
            unsigned int collision_ind = 0;
            //https://github.com/ros-visualization/rviz/blob/87742e4b8f40d8fe7e2a6952065651266faf73ba/src/rviz/robot/robot_link.cpp#L698
            for(std::vector<urdf::CollisionSharedPtr >::const_iterator vi = link->collision_array.begin(); vi != link->collision_array.end(); vi++, collision_ind++ )
            {
              urdf::CollisionConstSharedPtr collision = *vi;
              if( collision && collision->geometry )
              {
                std::shared_ptr<geometry_models::GeometryModel> model = getGeometry(tf_prefix, cam_model_, *link, *collision, collision_ind);
                
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
          
          visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers",5);
          
          
          return true;
          //TODO: Precompute all relevant transforms between links and camera
        }

//         void doPrecomputation(const cv_bridge::CvImage::ConstPtr& cv_image_ref)
//         {
//           
//         }
        
        
        
        // Header contains the frame to which transforms must be computed, subject to change
        bool CompositeModel::updateTransforms()
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
              
              
              //geometry_msgs::Vector3 offset = link_base_transform.transform.translation;
              
              //geometry_msgs::TransformStamped origin_base_transform;
              //tf2::doTransform(origin_link_transform, origin_base_transform, link_base_transform);
              
              //               tf2::Stamped<tf2::Transform> tf_origin_base_transform;
              //               tf2::fromMsg(origin_base_transform, tf_origin_base_transform);
              //               
              //               tf2::Transform tf_base_origin_transform = tf_origin_base_transform.inverse();
              //               
              //               geometry_msgs::TransformStamped base_origin_transform;
              //               base_origin_transform.transform = tf2::toMsg(tf_base_origin_transform);
              
              //current_transform = origin_base_transform;
              
              //geometry_msgs::TransformStamped origin_camera_transform;
              
              //tf2::doTransform(origin_base_transform, origin_camera_transform, base_optical_transform_);
              
              //current_transform = origin_camera_transform;
                
              //               offset_transform.transform.translation.x += offset.x;
              //               offset_transform.transform.translation.y += offset.y;
              //               offset_transform.transform.translation.z += offset.z;
              
              //geometry_msgs::Quaternion& quat = link_transform.transform.rotation;
              //quat.x=quat.y=quat.z=0;
              //quat.w=1;
              
              
              //tf2::doTransform(model->origin_transform_, model->current_transform_, link_transform);
              
              ROS_INFO_STREAM("Origin:Base = (" << model_frame_id << ":" << base_frame_id << ")= " << toString(origin_base_transform));
              
              //ROS_INFO_STREAM("Origin:Camera = (" << model_name << ":" << target_frame_id << ")= " << toString(origin_camera_transform));
              
//               geometry_msgs::PoseStamped test_pose, transformed_pose;
//               test_pose.pose.position.x = 1.2;
//               test_pose.pose.orientation.w = 1;
//               
//               tf2::doTransform(test_pose, transformed_pose, origin_camera_transform);
//               
//               ROS_INFO_STREAM("Transformed " << toString(test_pose.pose) << " to " << toString(transformed_pose.pose));
              
              
            } catch ( tf2::TransformException &ex ) {
              ROS_WARN_STREAM ("Problem finding transform:\n" <<ex.what() );
              all_good = false;
            }
            
          }
          return all_good;
        }
        
        cv::Rect CompositeModel::getColumnRect(const int x, const int y, const int width, const int height)
        {
          if(transpose)
          {
            return cv::Rect(y,x,height,width);
          }
          return cv::Rect(x,y,width,height);
        }
        
        cv::Rect CompositeModel::getColumnRect(const cv::Rect& rect)
        {
          return getColumnRect(rect.tl().x,rect.tl().y,rect.width,rect.height);
        }
        
        cv::Mat CompositeModel::generateHallucinatedRobotImpl(const geometry_msgs::Pose pose)
        {
          cv::Mat viz = HallucinatedRobotModelImpl::generateHallucinatedRobotImpl(pose);
          
          int img_width = cv_image_ref_->image.cols;
          int img_height = cv_image_ref_->image.rows;
          
          const std_msgs::Header header = base_optical_transform_.header;
          
          
          geometry_msgs::TransformStamped origin_trans;
          origin_trans.transform.rotation = pose.orientation;
          origin_trans.transform.translation.x = pose.position.x;
          origin_trans.transform.translation.y = pose.position.y;
          origin_trans.transform.translation.z = pose.position.z;
          
          geometry_msgs::PoseStamped pose_stamped;
          pose_stamped.pose = pose;
          
          ROS_DEBUG_STREAM("Pose: " << toString(pose));
          
          visualization_msgs::MarkerArray::Ptr markers = initMarkers(header, "hallucinated");
          
          for(std::shared_ptr<geometry_models::GeometryModel> model : models_)
          {

            
            geometry_msgs::TransformStamped model_pose_stamped;
            
            tf2::doTransform(model->current_transform_, model_pose_stamped, origin_trans);            
            
            ROS_DEBUG_STREAM("Pose of [" << model->frame_id_ << "] in Base frame: " << toString(model_pose_stamped));
            
            geometry_msgs::TransformStamped camera_pose_stamped;
            
            tf2::doTransform(model_pose_stamped, camera_pose_stamped, base_optical_transform_);
            
            
            ROS_DEBUG_STREAM("Pose of [" << model->frame_id_ << "] in Camera frame: " << toString(camera_pose_stamped));

            geometry_msgs::Pose model_pose;
            model_pose.position.x = camera_pose_stamped.transform.translation.x;
            model_pose.position.y = camera_pose_stamped.transform.translation.y;
            model_pose.position.z = camera_pose_stamped.transform.translation.z;
            
            //NOTE: Currently, only the position (not orientation) is transformed
            model_pose.orientation = pose.orientation; //camera_pose_stamped.transform.rotation;
            
            addMarker(markers, model_pose, *model, header, "hallucinated");
            
            std::vector<COLUMN_TYPE> cols = model->getColumns(model_pose, img_width, img_height);
            
            for(unsigned int i = 0; i < cols.size(); ++i)
            {
              cv::Rect roi = getColumnRect(cols[i].rect);
              cv::Mat col = cv::Mat(viz, roi); 
              float depth = cols[i].depth * scale_;
              //col.setTo(depth);
              col = cv::max(col,depth);
            }
            
          }
          
          visualization_pub_.publish(markers);
          
          if(transpose)
          {
            cv::Mat viz_t = viz.t();
          
            return viz_t;
          }
          
          return viz;
        }
        
        ComparisonResult CompositeModel::testCollisionImpl(const geometry_msgs::Pose pose, CCOptions options)
        {
          
          int img_width = cv_image_ref_->image.cols;
          int img_height = cv_image_ref_->image.rows;
          
          const std_msgs::Header header = base_optical_transform_.header;
          
          ComparisonResult result;
          
          geometry_msgs::TransformStamped origin_trans;
          origin_trans.transform.rotation = pose.orientation;
          origin_trans.transform.translation.x = pose.position.x;
          origin_trans.transform.translation.y = pose.position.y;
          origin_trans.transform.translation.z = pose.position.z;
          
          geometry_msgs::PoseStamped pose_stamped;
          pose_stamped.pose = pose;
          
          ROS_DEBUG_STREAM("Pose: " << toString(pose));
          
          visualization_msgs::MarkerArray::Ptr markers = initMarkers(header, "tested");
          
          for(std::shared_ptr<geometry_models::GeometryModel> model : models_)
          {

            
            geometry_msgs::TransformStamped model_pose_stamped;
            
            tf2::doTransform(model->current_transform_, model_pose_stamped, origin_trans);            
            
            ROS_DEBUG_STREAM("Pose of [" << model->frame_id_ << "] in Base frame: " << toString(model_pose_stamped));
            
            geometry_msgs::TransformStamped camera_pose_stamped;
            
            tf2::doTransform(model_pose_stamped, camera_pose_stamped, base_optical_transform_);
            
            
            ROS_DEBUG_STREAM("Pose of [" << model->frame_id_ << "] in Camera frame: " << toString(camera_pose_stamped));

            geometry_msgs::Pose model_pose;
            model_pose.position.x = camera_pose_stamped.transform.translation.x;
            model_pose.position.y = camera_pose_stamped.transform.translation.y;
            model_pose.position.z = camera_pose_stamped.transform.translation.z;
            
            //NOTE: Currently, only the position (not orientation) is transformed
            model_pose.orientation = pose.orientation; //camera_pose_stamped.transform.rotation;
            
            std::vector<COLUMN_TYPE> cols = model->getColumns(model_pose, img_width, img_height);
          
            for(unsigned int i = 0; i < cols.size(); ++i)
            {
              //TODO: limit rect to image size
              cv::Rect roi = getColumnRect(cols[i].rect);
              cv::Mat col = cv::Mat(this->image_ref_,roi); //cols[i].image;
              float depth = cols[i].depth * scale_;
              
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
          }
          
          if(transpose)
          {
            result.transpose();
          }
          
          return result;
        }
          
        ComparisonResult CompositeModel::isLessThan(const cv::Mat& col, float depth)
        {    
          return ::utils::isLessThan::vectorized(col, depth);
          
          return ::utils::isLessThan::stock(col, depth);
        }
        
        ComparisonResult CompositeModel::isLessThanDetails(const cv::Mat& col, float depth)
        {
          // TODO: replace 'show_im_' with more accurate variable name (ex: 'full_details' or something)
          if(show_im_)
          {
            ROS_DEBUG_STREAM_NAMED(name_,"FULL details!");
            return ::utils::isLessThan::fulldetails(col, depth);
          }
          else
          {
            return ::utils::isLessThan::details(col, depth);
          }
        }
        
        
        
        std::shared_ptr<geometry_models::GeometryModel> CompositeModel::getGeometry(const std::string& tf_prefix, std::shared_ptr<pips::utils::AbstractCameraModel> cam_model_, const urdf::Link& link, const urdf::Collision& collision, unsigned int collision_ind)
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
            model->cam_model_ = cam_model_;
            
            
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
        
        std::shared_ptr<geometry_models::GeometryModel> CompositeModel::getGeometry(const urdf::Cylinder& cylinder)
        {
          ROS_INFO_STREAM("Adding Cylinder primitive. Radius=" << cylinder.radius << ", length=" << cylinder.length);
          std::shared_ptr<geometry_models::Cylinder> model = std::make_shared<geometry_models::Cylinder>(cylinder.radius, cylinder.length);
          return model;
        }
        
        std::shared_ptr<geometry_models::GeometryModel> CompositeModel::getGeometry(const urdf::Box& box)
        {
          ROS_INFO_STREAM("Adding Box primitive. Length=" << box.dim.x << ", Width=" << box.dim.y << ", Height=" << box.dim.z);
          std::shared_ptr<geometry_models::Box> model = std::make_shared<geometry_models::Box>(box.dim.x, box.dim.y, box.dim.z);
          return model;
        }
        
        
        
        
        visualization_msgs::MarkerArray::Ptr CompositeModel::initMarkers(const std_msgs::Header& header, std::string ns)
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

        void CompositeModel::addMarker(visualization_msgs::MarkerArray::Ptr& markers, const geometry_msgs::Pose& pose, const geometry_models::GeometryModel& model, const std_msgs::Header& header, std::string ns)
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

        

    }
    
  }
  
}


// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "composite_model");
//   ros::NodeHandle nh;
//   ros::NodeHandle pnh("~");
//   pips::collision_testing::robot_models::CompositeModel model(nh,pnh);
//   model.init();
//   
//   ros::spin();
//   
// }

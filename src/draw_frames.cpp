#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

class FrameDrawer
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_,depthsub_;
  image_transport::Publisher pub_,depthpub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_;
  CvFont font_;
  bool firstFrame_,firstDepthFrame_;
  sensor_msgs::ImageConstPtr firstImMsg_,firstDepthImMsg_;
  tf::StampedTransform starting_frame_transform_,depth_starting_frame_transform_;
  tf2_ros::StaticTransformBroadcaster br;
  ros::Timer timer, depth_timer;
  std::vector<cv::Point3d> co_offsets_;

public:
  FrameDrawer(const std::vector<std::string>& frame_ids)
    : it_(nh_), frame_ids_(frame_ids), tf_listener_(tfBuffer_), firstFrame_(true), firstDepthFrame_(true)
  {
    std::string image_topic = nh_.resolveName("rgb_image");
    std::string depth_image_topic = nh_.resolveName("depth_image");
    sub_ = it_.subscribeCamera(image_topic, 10, &FrameDrawer::rgbImageCb, this);
    depthsub_ = it_.subscribeCamera(depth_image_topic, 10, &FrameDrawer::depthImageCb, this);
    pub_ = it_.advertise("rgb_image_out", 1);
    depthpub_ = it_.advertise("depth_image_out",1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

    double radius = .25;
    double height = .7;
    double clearance = .05;

    cv::Point3d topr(radius,-height,radius);
    cv::Point3d topl(-radius,-height,radius);
    cv::Point3d bottomr(radius,-clearance,radius);
    cv::Point3d bottoml(-radius,-clearance,radius);


    cv::Point3d offsets[] = {topr,topl,bottoml,bottomr};
    std::vector<cv::Point3d> co_offsets(offsets, offsets + sizeof(offsets) / sizeof(cv::Point3d) );
    co_offsets_ = co_offsets;
  }


  void rgbImageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {

    cam_model_.fromCameraInfo(info_msg);

    std::string stationary_frame("odom");
    std::string tracking_frame_id("base_link");
    std::string starting_frame_id("start_frame");
    ros::Duration timeout(1.0 / 30);


    if(firstFrame_) {

        try
        {
          geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform("odom", cam_model_.tfFrame(), ros::Time(0), timeout);
          transform.child_frame_id = "start_frame";

        ROS_ERROR("starting tf broadcast");
        br.sendTransform(transform);

        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          return;
        }

        firstImMsg_ = image_msg;
        firstFrame_ = false;
        ROS_ERROR("Saved first frame");
    }
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(firstImMsg_, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
      
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }


        geometry_msgs::TransformStamped transform;
        try
        {
          ros::Time acquisition_time = info_msg->header.stamp;
          transform = tfBuffer_.lookupTransform("start_frame", ros::Time(0), "base_link", ros::Time(0), "odom", timeout);
          
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("[draw_frames] TF exception:\n%s",ex.what());
          return;
        }


      geometry_msgs::Vector3 pt = transform.transform.translation;
      cv::Point3d pt_cv(pt.x, pt.y, pt.z);
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);

      static const int RADIUS = 3;
      cv::circle(image, uv, RADIUS, CV_RGB(255,0,0), -1);

      for (std::vector<cv::Point3d>::iterator it = co_offsets_.begin(); it != co_offsets_.end(); ++it) {
      
        cv::Point3d addedpnt = pt_cv + *it;
        uv = cam_model_.project3dToPixel(addedpnt);

      cv::circle(image, uv, RADIUS, CV_RGB(0,255,0), -1);
      }

    pub_.publish(input_bridge->toImageMsg());
  }


  void depthImageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {

    cam_model_.fromCameraInfo(info_msg);

    std::string stationary_frame("odom");
    std::string tracking_frame_id("base_link");
    std::string starting_frame_id("depth_start_frame");
    ros::Duration timeout(1.0 / 30);


    if(firstDepthFrame_) {

        try
        {
          geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform("odom", cam_model_.tfFrame(), ros::Time(0), timeout);
          transform.child_frame_id = "depth_start_frame";

        ROS_ERROR("starting depth tf broadcast");
        br.sendTransform(transform);

        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          return;
        }

        firstDepthImMsg_ = image_msg;
        firstDepthFrame_ = false;
        ROS_ERROR("Saved first depth frame");
    }
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(firstDepthImMsg_, sensor_msgs::image_encodings::TYPE_32FC1); //Note:if only comparing and not editing image, could use toCvShare to avoid any copying of data.
      image = input_bridge->image;
      
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }
      
      geometry_msgs::TransformStamped transform;
        try
        {
          ros::Time acquisition_time = info_msg->header.stamp;
          transform = tfBuffer_.lookupTransform("depth_start_frame", ros::Time(0), "base_link", ros::Time(0), "odom", timeout);
          
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("[draw_frames] TF exception:\n%s",ex.what());
          return;
        }


      geometry_msgs::Vector3 pt = transform.transform.translation;
      cv::Point3d pt_cv(pt.x, pt.y, pt.z);
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);

      static const int RADIUS = 3;
      cv::circle(image, uv, RADIUS, (2^16)-1, -1);

      std::vector<cv::Point2d> co_uv;
      double co_depth;

      for (std::vector<cv::Point3d>::iterator it = co_offsets_.begin(); it != co_offsets_.end(); ++it) {
      
        cv::Point3d addedpnt = pt_cv + *it;
        co_depth = addedpnt.z;
        uv = cam_model_.project3dToPixel(addedpnt);
        ROS_INFO("Coords: %f, %f", uv.x, uv.y);
        co_uv.push_back(uv);
    /*    if(uv.y <480 and uv.y >=0 and uv.x <640 and uv.x >=0)
          ROS_INFO("Intensity: %f", image.at<float>(uv.y, uv.x));
    */
      cv::circle(image, uv, RADIUS, (2^16)-1, -1);
      }
      double minXVal, maxXVal, minYVal,maxYVal;
      minYVal = cv::min(co_uv.at(0).y,co_uv.at(1).y);
      minXVal = cv::min(co_uv.at(1).x,co_uv.at(2).x);
      maxYVal = cv::max(co_uv.at(2).y,co_uv.at(3).y);
      maxXVal = cv::max(co_uv.at(3).x,co_uv.at(0).x);

/*      cv::Mat coords(co_uv);

      cv::minMaxLoc( coords.colRange(0,1), &minXVal, &maxXVal );
      cv::minMaxLoc( coords.colRange(1,2), &minYVal, &maxYVal );
      cv::Point2d topL(minXVal, minYVal);
      cv::Point2d bottomR(maxXVal, maxYVal);
*/
        ROS_INFO("Rectangle: %f, %f and %f, %f", co_uv.at(1).x, co_uv.at(1).y, co_uv.at(3).x, co_uv.at(3).y);
      cv::Rect roi(co_uv.at(1),co_uv.at(3));
 //     cv::Rect roi(minXVal, minYVal, maxXVal, maxYVal);

      cv::rectangle(image, roi, co_depth, CV_FILLED);

    depthpub_.publish(input_bridge->toImageMsg());
  }

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_frames");
  std::vector<std::string> frame_ids(argv + 1, argv + argc);
  FrameDrawer drawer(frame_ids);
  ros::spin();
}

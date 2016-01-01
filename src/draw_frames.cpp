#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>


class FrameDrawer
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_;
  CvFont font_;
  bool firstFrame_;
  sensor_msgs::ImageConstPtr firstImMsg_;
  tf::StampedTransform starting_frame_transform_;
  tf::TransformBroadcaster br;
  ros::Timer timer;
  std::vector<cv::Point3d> co_offsets_;

public:
  FrameDrawer(const std::vector<std::string>& frame_ids)
    : it_(nh_), frame_ids_(frame_ids), firstFrame_(true)
  {
    std::string image_topic = nh_.resolveName("image");
    sub_ = it_.subscribeCamera(image_topic, 10, &FrameDrawer::imageCb, this);
    pub_ = it_.advertise("image_out", 1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

    double radius = .25;
    double height = .7;
    double clearance = .05;

    cv::Point3d topr(radius,-height,radius);
    cv::Point3d topl(-radius,-height,radius);
    cv::Point3d bottomr(radius,-clearance,radius);
    cv::Point3d bottoml(-radius,-clearance,radius);
    cv::Point3d offsets[] = {topr,topl,bottomr,bottoml};
    std::vector<cv::Point3d> co_offsets(offsets, offsets + sizeof(offsets) / sizeof(cv::Point3d) );
    co_offsets_ = co_offsets;
  }

  void tfBroadcastCallBack(const ros::TimerEvent& event)
  {
     starting_frame_transform_.stamp_ = ros::Time::now();
     br.sendTransform(starting_frame_transform_);
  //      ROS_ERROR("sent tf");
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {

    cam_model_.fromCameraInfo(info_msg);

    std::string stationary_frame("odom");
    std::string tracking_frame_id("base_link");
    std::string starting_frame_id("start_frame");
    ros::Duration timeout(1.0 / 30);


    if(firstFrame_) {

        if(tf_listener_.waitForTransform("odom", cam_model_.tfFrame(), ros::Time(0), timeout))
        {
          tf_listener_.lookupTransform("odom", cam_model_.tfFrame(), ros::Time(0), starting_frame_transform_);
          starting_frame_transform_.child_frame_id_ = "start_frame";
          ros::NodeHandle nh;
        ROS_ERROR("starting timer");
          timer = nh.createTimer(ros::Duration(1.0/30), &FrameDrawer::tfBroadcastCallBack, this);

        }
        else
        {
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


      tf::StampedTransform transform;
      try {
        ros::Time acquisition_time = info_msg->header.stamp;

        tf_listener_.waitForTransform("start_frame", ros::Time(0), "base_link",
                                      ros::Time(0), "odom", timeout);
        tf_listener_.lookupTransform("start_frame", ros::Time(0), "base_link",
                                      ros::Time(0), "odom", transform);
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return;
      }

      tf::Point pt = transform.getOrigin();
      cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
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


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_frames");
  std::vector<std::string> frame_ids(argv + 1, argv + argc);
  FrameDrawer drawer(frame_ids);
  ros::spin();
}

#pragma once

#include "ImageSourceEngine.h"

#include <mutex>
#include <string>
#include <thread>
#if (!defined USING_CMAKE) && (defined _MSC_VER)
#ifdef _DEBUG
#pragma comment(lib, "libpxcmd_d")
#else
#pragma comment(lib, "libpxcmd")
#endif
#endif

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_broadcaster.h>
#include "../ITMLib/ITMLibDefines.h"
#include <queue>

namespace InputSource {

class RosImageSourceEngine : public BaseImageSourceEngine {
 private:
  cv_bridge::CvImagePtr cv_rgb_image_;
  cv_bridge::CvImagePtr cv_depth_image_;
  cv_bridge::CvImagePtr cv_mask_image_;

  //! ROS topic name for the incoming rgb messages.
  //std::string rgb_camera_info_topic_;
  //! ROS Topic name for the incoming depth messages.
  //std::string depth_camera_info_topic_;
  
  std::mutex mask_mutex_;
  std::mutex rgb_depth_mutex_;
  Vector2i image_size_rgb_, image_size_depth_, image_size_mask_;

  //ros::Subscriber rgb_sub_;
  //ros::Subscriber depth_sub_;
  ros::Subscriber mask_sub_;
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub_; 
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_; 
  // message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
  // message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  // ros::Subscriber tf_sub_;

  std::thread *spin_thread_;

  std::queue<cv_bridge::CvImagePtr> rgb_queue_;
  std::queue<cv_bridge::CvImagePtr> depth_queue_;
  std::queue<cv_bridge::CvImagePtr> mask_queue_;
  std::queue<std::pair<cv_bridge::CvImagePtr, cv_bridge::CvImagePtr> > rgb_depth_pair_queue_;

  double timestamp_sec_;

  double depth_scale_ = 1.0;

  bool dynamic_objects_ = false;

  /*!
   * Time stamp of the incoming images. This is used to synchronize the
   * incoming images with the pose estimation.
   */
  ros::Time depth_msg_time_stamp_;
  ros::NodeHandle nh_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol_;
  typedef message_filters::Synchronizer<sync_pol_> syncer_;
  boost::shared_ptr<syncer_> sync_;


 public:
  RosImageSourceEngine(ros::NodeHandle& nh, const char*& calibration_filename);

  ~RosImageSourceEngine();
  void maskCallback(const sensor_msgs::Image::ConstPtr& msg);
  void DepthRGBImagePairCallback(const sensor_msgs::ImageConstPtr& rgb,const sensor_msgs::ImageConstPtr& depth);
  void SpinROS();

  bool ImagePairMatches();
  double GetImageTimestamp();

  // ImageSourceEngine
  bool hasMoreImages(void) const;
  void getImages(ITMUChar4Image* rgb, ITMShortImage* raw_depth);
  void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth, ITMUCharImage *mask);
  Vector2i getDepthImageSize(void) const;
  Vector2i getRGBImageSize(void) const;
};

}  // namespace InfiniTAM

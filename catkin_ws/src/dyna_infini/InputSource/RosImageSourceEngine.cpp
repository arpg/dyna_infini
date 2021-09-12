// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "RosImageSourceEngine.h"

#include <cstdio>
#include <stdexcept>
#include <string>

#include "../ORUtils/FileUtils.h"


//#ifdef COMPILE_WITH_Ros

namespace InputSource {

RosImageSourceEngine::RosImageSourceEngine(ros::NodeHandle& nh,
                                           const char*& calibration_filename)
    : BaseImageSourceEngine(calibration_filename)
 {

  std::string rgb_image_topic;
  std::string depth_image_topic;
  std::string mask_image_topic;

  nh_ = nh;

  if(nh.getParam("rgb_image_topic", rgb_image_topic)){
    ROS_INFO("Get rgb image topic name %s \n",rgb_image_topic.c_str());
  }
  else{
    ROS_INFO("No rgb image topic name, use default /camera/rgb/image_color");
    rgb_image_topic = "/camera/rgb/image_color";
  }

  if(nh.getParam("depth_image_topic", depth_image_topic)){
    ROS_INFO("Get depth image topic name %s \n",depth_image_topic.c_str());
  }
  else{
    ROS_INFO("No depth image topic name, use default /camera/depth/image");
    depth_image_topic = "/camera/depth/image";
  }

  if(nh.getParam("mask_image_topic", mask_image_topic)){
    ROS_INFO("Get mask image topic name %s \n",mask_image_topic.c_str());
  }
  else{
    ROS_INFO("No depth image topic name, use default /camera/mask/image_raw");
    mask_image_topic = "/camera/mask/image_raw";
  }

	if(nh.getParam("dynamic_objects", dynamic_objects_)){
		ROS_INFO("has dynamic_objects or not ? %d \n", dynamic_objects_);
	}
	else{
		ROS_INFO("Not providing if we need to consider dynamic objects. Default not.  \n");
    dynamic_objects_ = false;
	}

  if(nh.getParam("depth_scale", depth_scale_)){
    ROS_INFO("Get depth scale %f. This should be the inverse of the first `affine` parameter in the calibration file \n", depth_scale_);
  }
  else{
    ROS_INFO("Use default depth scale 1.0");
  }

  rgb_sub_.subscribe(nh_,rgb_image_topic,10);
  depth_sub_.subscribe(nh_,depth_image_topic,10);
  
  sync_.reset(new syncer_(sync_pol_(10), rgb_sub_, depth_sub_) );
  sync_->registerCallback(boost::bind(&RosImageSourceEngine::DepthRGBImagePairCallback, this, _1, _2));
                                     

  if(dynamic_objects_){
    mask_sub_ = nh.subscribe(mask_image_topic, 10,
                                        &RosImageSourceEngine::maskCallback,
                                        this);
  }

  auto calib = this->getCalib();
  image_size_depth_ = calib.intrinsics_d.imgSize;
  image_size_rgb_   = calib.intrinsics_rgb.imgSize;
  //mask use rgb image size temporarily
  image_size_mask_  = calib.intrinsics_rgb.imgSize;


  spin_thread_ = new std::thread(&InputSource::RosImageSourceEngine::SpinROS,this);

}

RosImageSourceEngine::~RosImageSourceEngine() {
  delete spin_thread_;
}

void RosImageSourceEngine::DepthRGBImagePairCallback(const sensor_msgs::ImageConstPtr& rgb,const sensor_msgs::ImageConstPtr& depth){

  ROS_INFO_ONCE("Got rgb raw image and depth pair, push it into the queue.");
  cv_bridge::CvImagePtr cv_rgb_image;
  if(rgb->encoding == sensor_msgs::image_encodings::RGB8)
    cv_rgb_image = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::RGB8);
  else if(rgb->encoding == sensor_msgs::image_encodings::BGR8)
    cv_rgb_image = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
  else if(rgb->encoding == sensor_msgs::image_encodings::MONO8){
    //printf("get grayscale image and convert it to the 4 channel image since infinitam only accpet this \n");
    cv_rgb_image = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::MONO8);
    cv::cvtColor(cv_rgb_image->image, cv_rgb_image->image, CV_GRAY2RGB);
  }
  else{
    ROS_FATAL("unsupported rgb image encodings");
    ros::shutdown();
  }

  depth_msg_time_stamp_ = depth->header.stamp;
  cv_bridge::CvImagePtr cv_depth_image;

  if(depth->encoding != sensor_msgs::image_encodings::TYPE_32FC1 &&  depth->encoding != sensor_msgs::image_encodings::TYPE_16UC1){
    ROS_FATAL("unsupported depth image encodings");
    ros::shutdown();
  }
  cv_depth_image = cv_bridge::toCvCopy(depth, depth->encoding);

  //Add background  TEST ONLY
  int counter_nan = 0;
  int counter0 = 0;
  for(int i = 0; i<cv_depth_image->image.rows; i++){
    for(int j = 0; j<cv_depth_image->image.cols; j++){
      // if(cv_depth_image->image.ptr<float>(i)[j] == 0 ){
      //   //cv_depth_image->image.ptr<float>(i)[j] = 0.1;
      //   counter0++;
      // }
      if(isnan(cv_depth_image->image.ptr<float>(i)[j])){
        cv_depth_image->image.ptr<float>(i)[j] = 5.0;
        //counter_nan++;
      }
      // if(cv_depth_image->image.ptr<float>(i)[j] > 5.0)
      //   cv_depth_image->image.ptr<float>(i)[j] = 5.0;
    }
  }

  // If the image has 32FC1. Infinitam needs 16UCI.
  if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_depth_image->image)
        .convertTo(cv_depth_image->image, CV_16UC1, depth_scale_);
  }

  std::lock_guard<std::mutex> guard(rgb_depth_mutex_);
  rgb_depth_pair_queue_.push(std::make_pair(cv_rgb_image, cv_depth_image));

};


//we assume mask has the same timestamp with rgb image so we dont deal with its timestamp
void RosImageSourceEngine::maskCallback(const sensor_msgs::Image::ConstPtr& msg){
  ROS_INFO_ONCE("Got mask raw image. Push it into the queue");

  cv_bridge::CvImagePtr cv_mask_image;

  if(msg->encoding == sensor_msgs::image_encodings::TYPE_8UC1)
    cv_mask_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
  else if(msg->encoding == sensor_msgs::image_encodings::MONO8)
    cv_mask_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  else{
    ROS_FATAL("unsupported mask image encodings. Should be 8UC1 or MONO8, now it is %s", msg->encoding.c_str());
    ros::shutdown();  
  }

  std::lock_guard<std::mutex> guard(mask_mutex_);
  mask_queue_.push(cv_mask_image);
}


//convert ros image to infinitam version
void RosImageSourceEngine::getImages(ITMUChar4Image* rgb_image, ITMShortImage* raw_depth_image) {

  short* raw_depth_infinitam = raw_depth_image->GetData(MEMORYDEVICE_CPU);
  const uint16_t* depth_frame = reinterpret_cast<const uint16_t*>(cv_depth_image_->image.data);
  constexpr size_t depth_pixel_size = sizeof(uint16_t);
	static_assert(2 == depth_pixel_size, "sizeof(depth pixel) must equal 2");

	std::memcpy(raw_depth_infinitam, depth_frame, depth_pixel_size * raw_depth_image->noDims.x * raw_depth_image->noDims.y);

  Vector4u* rgb_infinitam = rgb_image->GetData(MEMORYDEVICE_CPU);

  cv::Size rgb_size = cv_rgb_image_->image.size();
  uint rgb_rows = rgb_size.height;
  uint rgb_cols = rgb_size.width;
  for (size_t i = 0; i < 3 * rgb_rows * rgb_cols; i += 3) {
    Vector4u pixel_value;
    pixel_value.r = cv_rgb_image_->image.data[i];
    pixel_value.g = cv_rgb_image_->image.data[i + 1];
    pixel_value.b = cv_rgb_image_->image.data[i + 2];
    pixel_value.w = 255;
    rgb_infinitam[i / 3] = pixel_value;
  }

};

void RosImageSourceEngine::getImages(ITMUChar4Image *rgb_image, ITMShortImage *raw_depth_image, ITMUCharImage *mask_image){
    //depth
    short* raw_depth_infinitam = raw_depth_image->GetData(MEMORYDEVICE_CPU);
    const uint16_t* depth_frame = reinterpret_cast<const uint16_t*>(cv_depth_image_->image.data);
    constexpr size_t depth_pixel_size = sizeof(uint16_t);
    static_assert(2 == depth_pixel_size, "sizeof(depth pixel) must equal 2");

    std::memcpy(raw_depth_infinitam, depth_frame, depth_pixel_size * raw_depth_image->noDims.x * raw_depth_image->noDims.y);

    //rgb. memcpy fails. Seems 4 channel direct copy has some problems
    Vector4u* rgb_infinitam = rgb_image->GetData(MEMORYDEVICE_CPU);

    cv::Size rgb_size = cv_rgb_image_->image.size();
    uint rgb_rows = rgb_size.height;
    uint rgb_cols = rgb_size.width;
    for (size_t i = 0; i < 3 * rgb_rows * rgb_cols; i += 3) {
      Vector4u pixel_value;
      pixel_value.r = cv_rgb_image_->image.data[i];
      pixel_value.g = cv_rgb_image_->image.data[i + 1];
      pixel_value.b = cv_rgb_image_->image.data[i + 2];
      pixel_value.w = 255;
      rgb_infinitam[i / 3] = pixel_value;
    }

    //mask
    uchar* mask_infinitam = mask_image->GetData(MEMORYDEVICE_CPU);
    const uchar* mask_frame = reinterpret_cast<const uchar*>(cv_mask_image_->image.data);
    constexpr size_t mask_pixel_size = sizeof(uchar);
    static_assert(1 == mask_pixel_size, "sizeof(depth pixel) must equal 1");
    //Assume rgb,depth and mask has the same row and column
    std::memcpy(mask_infinitam, mask_frame, mask_pixel_size * rgb_image->noDims.x * rgb_image->noDims.y);
};

bool RosImageSourceEngine::ImagePairMatches(){
  
  std::lock_guard<std::mutex> mask_guard(mask_mutex_);
  std::lock_guard<std::mutex> rgb_depth_guard(rgb_depth_mutex_);

  cv_rgb_image_ = rgb_depth_pair_queue_.front().first;
  cv_depth_image_ = rgb_depth_pair_queue_.front().second;
  if(dynamic_objects_) cv_mask_image_ = mask_queue_.front();

  //match the mask and the rgb first because the segmentation normally is later than the rgb
  if(dynamic_objects_){
    double time_im = cv_rgb_image_->header.stamp.toSec();
    double time_im_mask = cv_mask_image_->header.stamp.toSec();

    //cout<<"time is "<<std::setprecision(18)<<time_im_stereo<<","<<time_im_mask<<", difference "<<time_im_stereo-time_im_mask<<endl;
    double th_mask = 0.01;//use th_mask second as thershold to decide if the stereo image and mask are pairs

    //Check image pairs
    if(abs(time_im - time_im_mask) <= th_mask){
        rgb_depth_pair_queue_.pop();
        mask_queue_.pop();
    }
    else if(time_im - time_im_mask > th_mask){
        ROS_INFO("rgb/gray pair is in front of the mask. Should not often happen");
        while(time_im - time_im_mask > th_mask && !mask_queue_.empty()){
            mask_queue_.pop();
            if(!mask_queue_.empty())
                cv_mask_image_ = mask_queue_.front();
            else{
                ROS_WARN("Mask lags too much behind the stereo");
                return false;                
            }
            time_im_mask = cv_mask_image_->header.stamp.toSec();
        }

        //if the sequence is switched, it means the image is not well synchronized
        if(time_im_mask - time_im > th_mask){
            ROS_WARN("Image is not well synchronized. The localization result may not be good");
        }
    }
    else if(time_im_mask - time_im > th_mask){
        ROS_INFO("mask is in front of the stereo pair");
        while(time_im_mask - time_im > th_mask && !rgb_depth_pair_queue_.empty()){
            rgb_depth_pair_queue_.pop();
            if(!rgb_depth_pair_queue_.empty()){
              cv_rgb_image_ = rgb_depth_pair_queue_.front().first;
              cv_depth_image_ = rgb_depth_pair_queue_.front().second;              
            }
            else{
                ROS_WARN("RGB lags too much behind the mask");
                return false;                
            }
            time_im = cv_rgb_image_->header.stamp.toSec();;
        }

        if(time_im - time_im_mask > th_mask){
            ROS_WARN("Image is not well synchronized. The localization result may not be good");
        }
    }

    timestamp_sec_ = time_im;
  }

  return true;
}


bool RosImageSourceEngine::hasMoreImages(void) const{
  if(dynamic_objects_){
    if(depth_queue_.empty()|| rgb_queue_.empty() || mask_queue_.empty())
      return false;
  }
  else{
    if(depth_queue_.empty()|| rgb_queue_.empty())
      return false;    
  }

  return true;
}

double RosImageSourceEngine::GetImageTimestamp(){
  return timestamp_sec_;
}

Vector2i RosImageSourceEngine::getDepthImageSize(void) const {
  return image_size_depth_;
}
Vector2i RosImageSourceEngine::getRGBImageSize(void) const {
  return image_size_rgb_;
}

void RosImageSourceEngine::SpinROS(){
  ros::Rate r(500);
  printf("Execute ros spin in another thread........................ image \n");
  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
}

}  // namespace InfiniTAM


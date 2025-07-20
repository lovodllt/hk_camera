//
// Created by zihan on 2022/6/2.
//
#include <pluginlib/class_list_macros.h>
#include <hk_camera.h>
#include <utility>
#include <ros/time.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <fstream>
#include <string>
#include <std_msgs/Bool.h>

namespace hk_camera
{
PLUGINLIB_EXPORT_CLASS(hk_camera::HKCameraNodelet, nodelet::Nodelet)
HKCameraNodelet::HKCameraNodelet() : d_nh_(), d_it_(d_nh_)
{
}

void HKCameraNodelet::onInit()
{
  nh_ = this->getPrivateNodeHandle();
  node_name_ = nh_.getNamespace();
  ROS_INFO("node_name:%s",node_name_.substr(1).c_str());
  image_transport::ImageTransport it(nh_);
  pub_ = it.advertiseCamera("image_raw", 1);
  camera_raw_ = pub_.getTopic();
  this->status_change_srv_ = nh_.advertiseService("/exposure_status_switch", &HKCameraNodelet::changeStatusCB, this);

  nh_.param("camera_frame_id", image_.header.frame_id, std::string("camera_optical_frame"));
  nh_.param("camera_name", camera_name_, std::string("camera"));
  nh_.param("camera_info_url", camera_info_url_, std::string(""));
  nh_.param("image_width", image_width_, 1440);
  nh_.param("image_height", image_height_, 1080);
  nh_.param("image_offset_x", image_offset_x_, 0);
  nh_.param("image_offset_y", image_offset_y_, 0);
  nh_.param("pixel_format", pixel_format_, std::string("bgr8"));
  nh_.param("frame_id", frame_id_, std::string("camera_optical_frame"));
  nh_.param("camera_sn", camera_sn_, std::string(""));
  nh_.param("frame_rate", frame_rate_, 200.0);
  nh_.param("sleep_time", sleep_time_, 0);
  nh_.param("enable_imu_trigger", enable_imu_trigger_, false);
  nh_.param("imu_name", imu_name_, std::string("gimbal_imu"));
  nh_.param("gain_value", gain_value_, 15.0);
  nh_.param("gain_auto", gain_auto_, false);
  nh_.param("gamma_selector", gamma_selector_, 2);
  nh_.param("gamma_value", gamma_value_, 0.5);
  nh_.param("exposure_auto", exposure_auto_, true);
  nh_.param("exposure_value", exposure_value_, 20.0);
  nh_.param("exposure_max", exposure_max_, 3000.0);
  nh_.param("exposure_min", exposure_min_, 50.0);
  nh_.param("white_auto", white_auto_, true);
  nh_.param("white_selector", white_selector_, 0);
  nh_.param("enable_resolution", enable_resolution_, false);
  nh_.param("resolution_ratio_width", resolution_ratio_width_, 1440);
  nh_.param("resolution_ratio_height", resolution_ratio_height_, 1080);
  nh_.param("stop_grab", stop_grab_, false);
  nh_.param("is_fps_down", is_fps_down_, false);

  info_manager_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_));

  // check for default camera info
  if (!info_manager_->isCalibrated())
  {
    info_manager_->setCameraName(camera_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = image_.header.frame_id;
    camera_info.width = image_width_;
    camera_info.height = image_height_;
    info_manager_->setCameraInfo(camera_info);
  }
  ROS_INFO("Starting '%s' at %dx%d", camera_name_.c_str(), image_width_, image_height_);
  info_ = std::move(info_manager_->getCameraInfo());
  info_.header.frame_id = frame_id_;
  image_.header.frame_id = frame_id_;
  image_.height = image_height_;
  image_.width = image_width_;
  image_.step = image_width_ * 3;
  image_.data.resize(image_.height * image_.step);
  image_.encoding = pixel_format_;
  img_ = new unsigned char[image_.height * image_.step];

  initializeCamera();

  ros::NodeHandle p_nh(nh_, "hk_camera_reconfig");
  pub_rect_ = p_nh.advertise<sensor_msgs::Image>("/image_rect", 1);
  srv_ = new dynamic_reconfigure::Server<CameraConfig>(p_nh);
  dynamic_reconfigure::Server<CameraConfig>::CallbackType cb = boost::bind(&HKCameraNodelet::reconfigCB, this, _1, _2);
  srv_->setCallback(cb);
  if (enable_imu_trigger_)
  {
    imu_trigger_client_ = nh_.serviceClient<rm_msgs::EnableImuTrigger>("imu_trigger");
    rm_msgs::EnableImuTrigger imu_trigger_srv;
    imu_trigger_srv.request.imu_name = imu_name_;
    imu_trigger_srv.request.enable_trigger = true;
    while (!imu_trigger_client_.call(imu_trigger_srv))
    {
      ROS_WARN("Failed to call service enable_imu_trigger. Retry now.");
      ros::Duration(1).sleep();
    }
    if (imu_trigger_srv.response.is_success)
      ROS_INFO("Enable imu %s trigger camera successfully", imu_name_.c_str());
    else
      ROS_ERROR("Failed to enable imu %s trigger camera", imu_name_.c_str());
    enable_trigger_timer_ = nh_.createTimer(ros::Duration(0.5), &HKCameraNodelet::enableTriggerCB, this);
  }

  camera_change_sub = nh_.subscribe("/camera_name", 50, &hk_camera::HKCameraNodelet::cameraChange, this);
  camera_stop_sub_ = nh_.subscribe("/camera_stop", 50, &hk_camera::HKCameraNodelet::cameraStop, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), &HKCameraNodelet::timerCallback, this);
  ROS_INFO("Camera %s is ready", camera_name_.c_str());

}

void HKCameraNodelet::initializeCamera()
{
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  try
  {
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK)
      throw(nRet);
  }
  catch (int nRet)
  {
    std::cout << "MV_CC_EnumDevices fail! nRet " << std::hex << nRet << std::endl;
    exit(-1);
  }
  //  assert(MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList) == MV_OK);
  assert(stDeviceList.nDeviceNum > 0);

  // Opens the device.
  unsigned int nIndex = 0;
  MVCC_STRINGVALUE dev_sn;
  memset(&dev_sn, 0, sizeof(MVCC_STRINGVALUE));
  ros::Duration(sleep_time_).sleep();
  if (stDeviceList.nDeviceNum > 1)
  {
    for (; nIndex < stDeviceList.nDeviceNum; nIndex++)
    {
      assert(MV_CC_CreateHandle(&dev_handle_, stDeviceList.pDeviceInfo[nIndex]) == MV_OK);
      MV_CC_OpenDevice(dev_handle_);
      MV_CC_GetStringValue(dev_handle_, "DeviceSerialNumber", &dev_sn);
      if (strcmp(dev_sn.chCurValue, (char*)camera_sn_.data()) == 0)
      {
        break;
      }
      else
      {
        MV_CC_DestroyHandle(dev_handle_);
        if (nIndex == stDeviceList.nDeviceNum - 1)
          ROS_INFO("The serial number is false!");
      }
    }
  }
  else
  {
    assert(MV_CC_CreateHandle(&dev_handle_, stDeviceList.pDeviceInfo[nIndex]) == MV_OK);
    assert(MV_CC_OpenDevice(dev_handle_) == MV_OK);
    MV_CC_GetStringValue(dev_handle_, "DeviceSerialNumber", &dev_sn);
  }

  // Print the camera serial number
  ROS_INFO("Camera Serial Number: %s", dev_sn.chCurValue);

  // Retrieve and print the camera's model name using DeviceModelName
  MVCC_STRINGVALUE model_name;
  memset(&model_name, 0, sizeof(MVCC_STRINGVALUE));
  int nRet = MV_CC_GetStringValue(dev_handle_, "DeviceModelName", &model_name);
  if (nRet == MV_OK)
  {
    ROS_INFO("Camera Model: %s", model_name.chCurValue);
  }
  else
  {
    ROS_WARN("Failed to get camera model name. Error code: %x", nRet);
  }

  MvGvspPixelType format;
  if (pixel_format_ == "mono8")
    format = PixelType_Gvsp_Mono8;
  if (pixel_format_ == "mono16")
    format = PixelType_Gvsp_Mono16;
  if (pixel_format_ == "bgra8")
    format = PixelType_Gvsp_BayerBG8;
  if (pixel_format_ == "rgb8")
    format = PixelType_Gvsp_BayerRG8;
  if (pixel_format_ == "bgr8")
    format = PixelType_Gvsp_BayerGB8;
  if (format == 0)
    static_assert(true, "Illegal format");

  //  assert(MV_CC_SetEnumValue(dev_handle_,"PixelFormat",format) == MV_OK);
  assert(MV_CC_SetIntValue(dev_handle_, "Width", image_width_) == MV_OK);
  assert(MV_CC_SetIntValue(dev_handle_, "Height", image_height_) == MV_OK);
  assert(MV_CC_SetIntValue(dev_handle_, "OffsetX", image_offset_x_) == MV_OK);
  assert(MV_CC_SetIntValue(dev_handle_, "OffsetY", image_offset_y_) == MV_OK);
  //  AcquisitionLineRate ,LineRate can't be set
  //  assert(MV_CC_SetBoolValue(dev_handle_,"AcquisitionLineRateEnable", true)== MV_OK);
  //  assert(MV_CC_SetIntValue(dev_handle_,"AcquisitionLineRate", 10)== MV_OK);

  _MVCC_FLOATVALUE_T frame_rate;
  MV_CC_SetFrameRate(dev_handle_, frame_rate_);
  MV_CC_GetFrameRate(dev_handle_, &frame_rate);
  ROS_INFO("Frame rate is: %f", frame_rate.fCurValue);

  if (enable_imu_trigger_)
  {
    assert(MV_CC_SetEnumValue(dev_handle_, "TriggerMode", 1) == MV_OK);
    assert(MV_CC_SetEnumValue(dev_handle_, "TriggerSource", MV_TRIGGER_SOURCE_LINE2) == MV_OK);
    assert(MV_CC_SetEnumValue(dev_handle_, "TriggerActivation", 2) == MV_OK);
    //      Raising_filter_value Setting haven't been realized

    trigger_sub_ =
        nh_.subscribe("/rm_hw/" + imu_name_ + "/trigger_time", 50, &hk_camera::HKCameraNodelet::triggerCB, this);
  }
  else
  {
    assert(MV_CC_SetEnumValue(dev_handle_, "TriggerMode", 0) == MV_OK);
  }
  MV_CC_RegisterImageCallBackEx(dev_handle_, onFrameCB, dev_handle_);

  if (MV_CC_StartGrabbing(dev_handle_) == MV_OK)
  {
    ROS_INFO("Stream On.");
  }
}

void HKCameraNodelet::timerCallback(const ros::TimerEvent&) {
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  try
  {
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK)
      throw(nRet);
  }
  catch (int nRet)
  {
    std::cout << "MV_CC_EnumDevices fail! nRet " << std::hex << nRet << std::endl;
    exit(-1);
  }
  if (stDeviceList.nDeviceNum == 0)
    camera_restart_flag_ = true;
  if (camera_restart_flag_ && stDeviceList.nDeviceNum > 0)
  {
    initializeCamera();
    camera_restart_flag_ = false;
  }
}

bool HKCameraNodelet::changeStatusCB(rm_msgs::StatusChange::Request& change, rm_msgs::StatusChange::Response& res)
{
  if (change.target)
    nh_.param("exposure_value_windmill", exposure_value_, 20.0);
  else
    nh_.param("exposure_value", exposure_value_, 20.0);
  assert(MV_CC_SetEnumValue(dev_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF) == MV_OK);
  assert(MV_CC_SetFloatValue(dev_handle_, "ExposureTime", exposure_value_) == MV_OK);
  res.switch_is_success = true;
  return true;
}

void HKCameraNodelet::cameraChange(const std_msgs::String camera_change)
{
  if (strcmp(camera_change.data.c_str(), node_name_.substr(1).c_str()) == 0)
    MV_CC_StartGrabbing(dev_handle_);
  else
    MV_CC_StopGrabbing(dev_handle_);
}

void HKCameraNodelet::cameraStop(const std_msgs::Bool camera_stop_msg_)
{
  if (camera_stop_msg_.data == true && strcmp("camera_back", node_name_.substr(1).c_str()) == 0)
    MV_CC_StopGrabbing(dev_handle_);
  else
    MV_CC_StartGrabbing(dev_handle_);
}

void HKCameraNodelet::triggerCB(const sensor_msgs::TimeReference::ConstPtr& time_ref)
{
  last_trigger_time_ = time_ref->time_ref;
  hk_camera::TriggerPacket pkt;
  pkt.trigger_time_ = time_ref->time_ref;
  pkt.trigger_counter_ = time_ref->header.seq;
  fifoWrite(pkt);
}

void HKCameraNodelet::enableTriggerCB(const ros::TimerEvent&)
{
  if ((ros::Time::now() - last_trigger_time_).toSec() > 1.0)
  {
    ROS_INFO("Try to enable imu %s to trigger camera.", imu_name_.c_str());
    rm_msgs::EnableImuTrigger imu_trigger_srv;
    imu_trigger_srv.request.imu_name = imu_name_;
    imu_trigger_srv.request.enable_trigger = true;
    imu_trigger_client_.call(imu_trigger_srv);
    if (trigger_not_sync_)
      trigger_not_sync_ = false;
  }
}

void HKCameraNodelet::fifoWrite(TriggerPacket pkt)
{
  if (fifo_front_ == (fifo_rear_ + 1) % FIFO_SIZE)
  {
    ROS_WARN("FIFO overflow!");
    return;
  }
  fifo_[fifo_rear_] = pkt;
  fifo_rear_ = (fifo_rear_ + 1) % FIFO_SIZE;
}

bool HKCameraNodelet::fifoRead(TriggerPacket& pkt)
{
  if (fifo_front_ == fifo_rear_)
    return false;
  pkt = fifo_[fifo_front_];
  fifo_front_ = (fifo_front_ + 1) % FIFO_SIZE;
  return true;
}

void HKCameraNodelet::onFrameCB(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
  if (pFrameInfo)
  {
    ros::Time now = ros::Time::now();
    if (enable_imu_trigger_)
    {
      if (!trigger_not_sync_)
      {
        TriggerPacket pkt;
        while (!fifoRead(pkt))
        {
          ros::Duration(0.001).sleep();
        }
        //        ROS_INFO("imu:%f", now.toSec() - pkt.trigger_time_.toSec());
        if (pkt.trigger_counter_ != receive_trigger_counter_++)
        {
          ROS_WARN("Trigger not in sync!");
          trigger_not_sync_ = true;
        }
//        else if ((now - pkt.trigger_time_).toSec() < 0)
//        {
//          ROS_WARN("Trigger not in sync! Maybe any CAN frames have be dropped?");
//          trigger_not_sync_ = true;
//        }
        else if ((now - pkt.trigger_time_).toSec() > 0.013)
        {
          ROS_WARN("Trigger not in sync! Maybe imu %s does not actually trigger camera?", imu_name_.c_str());
          trigger_not_sync_ = true;
        }
        else
        {
          image_.header.stamp = pkt.trigger_time_;
          info_.header.stamp = pkt.trigger_time_;
        }
      }
      if (trigger_not_sync_)
      {
        fifo_front_ = fifo_rear_;
        rm_msgs::EnableImuTrigger imu_trigger_srv;
        imu_trigger_srv.request.imu_name = imu_name_;
        imu_trigger_srv.request.enable_trigger = false;
        imu_trigger_client_.call(imu_trigger_srv);
        ROS_INFO("Disable imu %s from triggering camera.", imu_name_.c_str());
        receive_trigger_counter_ = fifo_[fifo_rear_ - 1].trigger_counter_ + 1;
        return;
      }
    }
    else
    {
      ros::Time now = ros::Time::now();
      image_.header.stamp = now;
      info_.header.stamp = now;
    }

    MV_CC_PIXEL_CONVERT_PARAM stConvertParam = { 0 };
    // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format,
    // destination pixel format, output data buffer, provided output buffer size
    stConvertParam.nWidth = pFrameInfo->nWidth;
    stConvertParam.nHeight = pFrameInfo->nHeight;
    stConvertParam.pSrcData = pData;
    stConvertParam.nSrcDataLen = pFrameInfo->nFrameLen;
    stConvertParam.enSrcPixelType = pFrameInfo->enPixelType;
    stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    stConvertParam.pDstBuffer = img_;
    stConvertParam.nDstBufferSize = pFrameInfo->nWidth * pFrameInfo->nHeight * 3;
    MV_CC_ConvertPixelType(dev_handle_, &stConvertParam);
    memcpy((char*)(&image_.data[0]), img_, image_.step * image_.height);

    //      if(take_photo_)
    //      {
    //          std::string str;
    //          str = std::to_string(count_);
    //          ROS_INFO("ok");
    //          cv_bridge::CvImagePtr cv_ptr1;
    //          cv_ptr1 = cv_bridge::toCvCopy(image_, "bgr8");
    //          cv::Mat cv_img1;
    //          cv_ptr1->image.copyTo(cv_img1);
    //          cv::imwrite("/home/irving/carphoto/"+str+".jpg",cv_img1);
    //          count_++;
    //      }

    if (enable_resolution_)
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(image_, "bgr8");
      cv::Mat cv_img;
      cv_ptr->image.copyTo(cv_img);
      sensor_msgs::ImagePtr image_rect_ptr;

      cv::resize(cv_img, cv_img, cvSize(resolution_ratio_width_, resolution_ratio_height_));
      image_rect_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_img).toImageMsg();
      pub_rect_.publish(image_rect_ptr);

      //    if (strcmp(camera_name_.data(), "hk_right"))
      //    {
      //      cv::Rect rect(0, 0, 1440 - width_, 1080);
      //      cv_img = cv_img(rect);
      //      image_rect_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_img).toImageMsg();
      //      pub_rect_.publish(image_rect_ptr);
      //    }
      //    if (strcmp(camera_name_.data(), "hk_left"))
      //    {
      //      cv::Rect rect(width_, 0, 1440 - width_, 1080);
      //      cv_img = cv_img(rect);
      //      image_rect_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_img).toImageMsg();
      //      pub_rect_.publish(image_rect_ptr);
      //    }
    }
    pub_.publish(image_, info_);
  }
  else
    ROS_ERROR("Grab image failed!");
}

void HKCameraNodelet::reconfigCB(CameraConfig& config, uint32_t level)
{
  (void)level;

  // Launch setting
  if (initialize_flag_)
  {
    config.exposure_auto = exposure_auto_;
    config.exposure_value = exposure_value_;
    config.exposure_max = exposure_max_;
    config.exposure_min = exposure_min_;
    config.gain_auto = gain_auto_;
    config.gain_value = gain_value_;
    config.gamma_selector = gamma_selector_;
    config.gamma_value = gamma_value_;
    config.white_auto = white_auto_;
    config.white_selector = white_selector_;
    config.stop_grab = stop_grab_;
    initialize_flag_ = false;
  }

  // Switch camera
  if (!config.stop_grab)
    MV_CC_StartGrabbing(dev_handle_);
  else
    MV_CC_StopGrabbing(dev_handle_);

  // Exposure
  if (config.exposure_auto)
  {
    _MVCC_FLOATVALUE_T exposure_time;
//    assert(MV_CC_SetIntValue(dev_handle_, "AutoExposureTimeLowerLimit", config.exposure_min) == MV_OK);
    MV_CC_SetIntValue(dev_handle_, "AutoExposureTimeLowerLimit", config.exposure_min);
    assert(MV_CC_SetIntValue(dev_handle_, "AutoExposureTimeUpperLimit", config.exposure_max) == MV_OK);
    assert(MV_CC_SetEnumValue(dev_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_CONTINUOUS) == MV_OK);
    assert(MV_CC_GetFloatValue(dev_handle_, "ExposureTime", &exposure_time) == MV_OK);
    config.exposure_value = exposure_time.fCurValue;
  }
  else
  {
    assert(MV_CC_SetEnumValue(dev_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF) == MV_OK);
    assert(MV_CC_SetFloatValue(dev_handle_, "ExposureTime", config.exposure_value) == MV_OK);
  }

  // Gain
  if (config.gain_auto)
  {
    _MVCC_FLOATVALUE_T gain_value;
    assert(MV_CC_SetFloatValue(dev_handle_, "AutoGainLowerLimit", config.gain_min) == MV_OK);
    assert(MV_CC_SetFloatValue(dev_handle_, "AutoGainUpperLimit", config.gain_max) == MV_OK);
    assert(MV_CC_SetEnumValue(dev_handle_, "GainAuto", MV_GAIN_MODE_CONTINUOUS) == MV_OK);
    assert(MV_CC_GetFloatValue(dev_handle_, "Gain", &gain_value) == MV_OK);
    config.gain_value = gain_value.fCurValue;
  }
  else
  {
    _MVCC_FLOATVALUE_T gain_value;
    assert(MV_CC_SetEnumValue(dev_handle_, "GainAuto", MV_GAIN_MODE_OFF) == MV_OK);
    assert(MV_CC_SetFloatValue(dev_handle_, "Gain", config.gain_value) == MV_OK);
    assert(MV_CC_GetFloatValue(dev_handle_, "Gain", &gain_value) == MV_OK);
    config.gain_value = gain_value.fCurValue;
  }

  // Black level
  // Can not be used!
  assert(MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF) == MV_OK);
  switch (config.white_selector)
  {
    case 0:
      assert(MV_CC_SetEnumValue(dev_handle_, "BalanceRatioSelector", 0) == MV_OK);
      break;
    case 1:
      assert(MV_CC_SetEnumValue(dev_handle_, "BalanceRatioSelector", 1) == MV_OK);
      break;
    case 2:
      assert(MV_CC_SetEnumValue(dev_handle_, "BalanceRatioSelector", 2) == MV_OK);
      break;
  }

  _MVCC_INTVALUE_T white_value;
  if (config.white_auto)
  {
    assert(MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS) == MV_OK);
    assert(MV_CC_GetIntValue(dev_handle_, "BalanceRatio", &white_value) == MV_OK);
    config.white_value = white_value.nCurValue;
  }
  else
  {
    assert(MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF) == MV_OK);
    assert(MV_CC_GetIntValue(dev_handle_, "BalanceRatio", &white_value) == MV_OK);
    config.white_value = white_value.nCurValue;
  }

  switch (config.gamma_selector)
  {
    case 0:
      assert(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", true) == MV_OK);
      assert(MV_CC_SetEnumValue(dev_handle_, "GammaSelector", MV_GAMMA_SELECTOR_SRGB) == MV_OK);
      break;
    case 1:
      assert(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", true) == MV_OK);
      assert(MV_CC_SetEnumValue(dev_handle_, "GammaSelector", MV_GAMMA_SELECTOR_USER) == MV_OK);
      assert(MV_CC_SetGamma(dev_handle_, config.gamma_value) == MV_OK);
      break;
    case 2:
//      assert(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", false) == MV_OK);
      MV_CC_SetBoolValue(dev_handle_, "GammaEnable", false);
      auto gamma_status = MV_CC_SetBoolValue(dev_handle_, "GammaEnable", false);
//      if (gamma_status != MV_OK) {
//        NODELET_ERROR("Failed to set GammaEnable: %d", gamma_status);
//      } else {
//        NODELET_INFO("Successfully set GammaEnable");
//      }
      break;
  }

  take_photo_ = config.take_photo;

  is_fps_down_ = config.is_fps_down;
  if (is_fps_down_)
  {
    ROS_INFO("Fps down mode is on.");
    FpsDown();
  }
  else
  {
    ROS_INFO("Fps down mode is off.");
    d_sub_.shutdown();
    d_pub_.shutdown();
  }
  //  Width offset of image
  //  width_ = config.width_offset;
}

HKCameraNodelet::~HKCameraNodelet()
{
  MV_CC_StopGrabbing(dev_handle_);
  MV_CC_DestroyHandle(dev_handle_);
}

void* HKCameraNodelet::dev_handle_;
unsigned char* HKCameraNodelet::img_;
sensor_msgs::Image HKCameraNodelet::image_;
sensor_msgs::Image HKCameraNodelet::image_rect;
image_transport::CameraPublisher HKCameraNodelet::pub_;
ros::Publisher HKCameraNodelet::pub_rect_;
sensor_msgs::CameraInfo HKCameraNodelet::info_;
int HKCameraNodelet::width_{};
std::string HKCameraNodelet::imu_name_;
std::string HKCameraNodelet::camera_name_;
ros::ServiceClient HKCameraNodelet::imu_trigger_client_;
bool HKCameraNodelet::enable_imu_trigger_;
bool HKCameraNodelet::trigger_not_sync_ = false;
const int HKCameraNodelet::FIFO_SIZE = 1023;
int HKCameraNodelet::fifo_front_ = 0;
int HKCameraNodelet::fifo_rear_ = 0;
bool HKCameraNodelet::take_photo_{};
struct TriggerPacket HKCameraNodelet::fifo_[FIFO_SIZE];
uint32_t HKCameraNodelet::receive_trigger_counter_ = 0;
bool HKCameraNodelet::enable_resolution_ = false;
int HKCameraNodelet::resolution_ratio_width_ = 1440;
int HKCameraNodelet::resolution_ratio_height_ = 1080;
bool HKCameraNodelet::camera_restart_flag_{};

void HKCameraNodelet::FpsDown()
{
  d_sub_ = d_it_.subscribe(camera_raw_, 10, &HKCameraNodelet::imageCallback, this);
  d_pub_ = d_it_.advertise("/hk_camera/image_raw_down", 10);
  target_fps_ = 40;
  last_pub_time_ = ros::Time::now();
}

void HKCameraNodelet::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ros::Time current_time = ros::Time::now();
  double elapsed_time = (current_time - last_pub_time_).toSec();
  double target_interval = 1.0 / target_fps_;
  if (elapsed_time >= target_interval)
  {
    d_pub_.publish(msg);
    last_pub_time_ = current_time;
  }
}

}  // namespace hk_camera

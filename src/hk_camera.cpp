//
// Created by zihan on 2022/6/2.
//
#include <pluginlib/class_list_macros.h>
#include <hk_camera.h>
#include <utility>
#include <ros/time.h>

namespace hk_camera
{
PLUGINLIB_EXPORT_CLASS(hk_camera::HKCameraNodelet, nodelet::Nodelet)
HKCameraNodelet::HKCameraNodelet()
{
}

void HKCameraNodelet::onInit()
{
  nh_ = this->getPrivateNodeHandle();
  image_transport::ImageTransport it(nh_);
  pub_ = it.advertiseCamera("image_raw", 1);

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
  nh_.param("frame_rate", frame_rate_, 70);
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

  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  assert(MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE , &stDeviceList) == MV_OK);
  assert(stDeviceList.nDeviceNum > 0);

  // Opens the device.
  unsigned int nIndex = 0;
  MVCC_STRINGVALUE dev_sn;
  memset(&dev_sn, 0, sizeof(MVCC_STRINGVALUE));
  if(stDeviceList.nDeviceNum > 1) {
      for (; nIndex < stDeviceList.nDeviceNum; nIndex++) {
          assert(MV_CC_CreateHandle(&dev_handle_, stDeviceList.pDeviceInfo[nIndex]) == MV_OK);
          sleep(1);
          MV_CC_OpenDevice(dev_handle_);
          MV_CC_GetStringValue(dev_handle_, "DeviceSerialNumber", &dev_sn);
          if (strcmp(dev_sn.chCurValue, (char *) camera_sn_.data()) == 0) {
              break;
          } else {
              MV_CC_DestroyHandle(dev_handle_);
              if (nIndex == stDeviceList.nDeviceNum - 1)
                  ROS_INFO("The serial number is false!");
          }
      }
  }
  else{
      assert(MV_CC_CreateHandle(&dev_handle_, stDeviceList.pDeviceInfo[nIndex]) == MV_OK);
      assert(MV_CC_OpenDevice(dev_handle_)==MV_OK);
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
  assert(MV_CC_SetIntValue(dev_handle_,"Width",image_width_) == MV_OK);
  assert(MV_CC_SetIntValue(dev_handle_,"Height",image_height_) == MV_OK);
  assert(MV_CC_SetIntValue(dev_handle_,"OffsetX",image_offset_x_) == MV_OK);
  assert(MV_CC_SetIntValue(dev_handle_,"OffsetY",image_offset_y_) == MV_OK);
//  AcquisitionLineRate ,LineRate ,FrameRate can't be set
//  assert(MV_CC_SetBoolValue(dev_handle_,"AcquisitionLineRateEnable", true)== MV_OK);
//  assert(MV_CC_SetIntValue(dev_handle_,"AcquisitionLineRate", 10)== MV_OK);
  _MVCC_FLOATVALUE_T a;
  MV_CC_SetFrameRate(dev_handle_,frame_rate_);
  MV_CC_GetFrameRate(dev_handle_,&a);
    ROS_INFO("Frame rate is: %f",a.fCurValue);
  MV_CC_RegisterImageCallBackEx(dev_handle_, onFrameCB, dev_handle_);

  if (MV_CC_StartGrabbing(dev_handle_) == MV_OK)
  {
    ROS_INFO("Stream On.");
  }

  ros::NodeHandle p_nh(nh_, "hk_camera_reconfig");
  srv_ = new dynamic_reconfigure::Server<CameraConfig>(p_nh);
  dynamic_reconfigure::Server<CameraConfig>::CallbackType cb =
      boost::bind(&HKCameraNodelet::reconfigCB, this, _1, _2);
  srv_->setCallback(cb);
}


void HKCameraNodelet::onFrameCB(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
  if (pFrameInfo)
  {
    ros::Time now = ros::Time::now();
    image_.header.stamp = now;
    info_.header.stamp = now;

    MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
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
    pub_.publish(image_, info_);
  }
  else
    ROS_ERROR("Grab image failed!");
}

void HKCameraNodelet::reconfigCB(CameraConfig& config, uint32_t level)
{
  (void)level;
  // Exposure
  if (config.exposure_auto)
  {
    _MVCC_FLOATVALUE_T exposure_time;
    assert(MV_CC_SetIntValue(dev_handle_,"AutoExposureTimeLowerLimit",config.exposure_min)==MV_OK);
    MV_CC_SetIntValue(dev_handle_,"AutoExposureTimeUpperLimit",config.exposure_max);
    MV_CC_SetEnumValue(dev_handle_,"ExposureAuto",MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
    MV_CC_GetFloatValue(dev_handle_,"ExposureTime",&exposure_time);
    config.exposure_value = exposure_time.fCurValue;
  }
  else
  {
    MV_CC_SetEnumValue(dev_handle_,"ExposureAuto",MV_EXPOSURE_AUTO_MODE_OFF);
    assert(MV_CC_SetFloatValue(dev_handle_,"ExposureTime",config.exposure_value)==MV_OK);
  }

  // Gain
  if (config.gain_auto)
  {
    _MVCC_FLOATVALUE_T gain_value;
    assert(MV_CC_SetFloatValue(dev_handle_,"AutoGainLowerLimit",config.gain_min)==MV_OK);
    MV_CC_SetFloatValue(dev_handle_,"AutoGainUpperLimit",config.gain_max);
    MV_CC_SetEnumValue(dev_handle_,"GainAuto",MV_GAIN_MODE_CONTINUOUS);
    MV_CC_GetFloatValue(dev_handle_,"Gain",&gain_value);
    config.gain_value = gain_value.fCurValue;
  }
  else
  {
    MV_CC_SetEnumValue(dev_handle_,"GainAuto",MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(dev_handle_,"Gain",config.gain_value);

  }

  // Black level
  // Can not be used!
  assert(MV_CC_SetEnumValue(dev_handle_,"BalanceWhiteAuto",MV_BALANCEWHITE_AUTO_OFF ) == MV_OK);
  switch (config.white_selector)
  {
      case 0:
          assert(MV_CC_SetEnumValue(dev_handle_,"BalanceRatioSelector",0)==MV_OK);
          break;
      case 1:
          assert(MV_CC_SetEnumValue(dev_handle_,"BalanceRatioSelector",1)==MV_OK);
          break;
      case 2:
          assert(MV_CC_SetEnumValue(dev_handle_,"BalanceRatioSelector",2)==MV_OK);
          break;
  }

    _MVCC_INTVALUE_T white_value;
  if (config.white_auto)
  {
    assert(MV_CC_SetEnumValue(dev_handle_,"BalanceWhiteAuto",MV_BALANCEWHITE_AUTO_CONTINUOUS ) == MV_OK);
    assert(MV_CC_GetIntValue(dev_handle_,"BalanceRatio",&white_value) == MV_OK);
    config.white_value = white_value.nCurValue;
  }
  else
  {
    assert(MV_CC_SetEnumValue(dev_handle_,"BalanceWhiteAuto",MV_BALANCEWHITE_AUTO_OFF ) == MV_OK);
    assert(MV_CC_GetIntValue(dev_handle_,"BalanceRatio",&white_value) == MV_OK);
    config.white_value = white_value.nCurValue;
  }

  if(config.gamma_enable) {
      assert(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", true)==MV_OK);
      assert(MV_CC_SetEnumValue(dev_handle_, "GammaSelector", MV_GAMMA_SELECTOR_USER)==MV_OK);
      MV_CC_GetGamma(dev_handle_, &gamma_value);
      if(MV_CC_SetGamma(dev_handle_,config.gain_value)==MV_E_GC_RANGE)
      {
          MV_CC_SetGamma(dev_handle_,gamma_value.fCurValue);
      }
  }
  else{
      assert(MV_CC_SetEnumValue(dev_handle_, "GammaSelector", MV_GAMMA_SELECTOR_SRGB)==MV_OK);
  }
}

HKCameraNodelet::~HKCameraNodelet()
{
  MV_CC_StopGrabbing(dev_handle_);
  MV_CC_DestroyHandle(dev_handle_);
}

void* HKCameraNodelet::dev_handle_;
unsigned char* HKCameraNodelet::img_;
sensor_msgs::Image HKCameraNodelet::image_;
image_transport::CameraPublisher HKCameraNodelet::pub_;
sensor_msgs::CameraInfo HKCameraNodelet::info_;

}  // namespace hk_camera

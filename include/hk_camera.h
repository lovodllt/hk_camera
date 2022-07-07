//
// Created by zihan on 2022/6/2.
//

#ifndef SRC_HK_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#define SRC_HK_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <hk_camera/CameraConfig.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/TimeReference.h>
#include <rm_msgs/CameraStatus.h>
#include <string>
#include "libMVSapi/MvCameraControl.h"

namespace hk_camera
{
class HKCameraNodelet : public nodelet::Nodelet
{
public:
  HKCameraNodelet();
  ~HKCameraNodelet() override;

  void onInit() override;
  static sensor_msgs::Image image_;
  static sensor_msgs::Image image_rect;

private:
  void reconfigCB(CameraConfig& config, uint32_t level);

  ros::NodeHandle nh_;
  static void* dev_handle_;
  dynamic_reconfigure::Server<CameraConfig>* srv_{};

  boost::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  static std::string camera_name_;
  std::string camera_info_url_, pixel_format_, frame_id_, camera_sn_;
  int frame_rate_;
  int image_width_{}, image_height_{}, image_offset_x_{}, image_offset_y_{}, sleep_time_{};
  static int width_;
  static unsigned char* img_;
  static image_transport::CameraPublisher pub_;
  static ros::Publisher pub_rect_;
  static sensor_msgs::CameraInfo info_;
  static void __stdcall onFrameCB(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
};
}  // namespace hk_camera

#endif  // SRC_HK_CAMERA_INCLUDE_GALAXY_CAMERA_H_

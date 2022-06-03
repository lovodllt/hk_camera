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
struct TriggerPacket
{
  uint32_t trigger_counter_;
  ros::Time trigger_time_;
};
class HKCameraNodelet : public nodelet::Nodelet
{
public:
  HKCameraNodelet();
  ~HKCameraNodelet() override;

  void onInit() override;
  static sensor_msgs::Image image_;

private:
  void reconfigCB(CameraConfig& config, uint32_t level);
  void triggerCB(const sensor_msgs::TimeReference::ConstPtr& time_ref);

  ros::NodeHandle nh_;
  static void* dev_handle_;
  dynamic_reconfigure::Server<CameraConfig>* srv_{};
//  int last_channel_ = 0;

  boost::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  std::string camera_name_, camera_info_url_, pixel_format_, frame_id_, camera_sn_;
  int image_width_{}, image_height_{}, image_offset_x_{}, image_offset_y_{}, raising_filter_value_{};
  static bool enable_imu_trigger_;
  static unsigned char* img_;
  static image_transport::CameraPublisher pub_;
  static sensor_msgs::CameraInfo info_;
  static void __stdcall onFrameCB(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);

  static bool device_open_;
  static bool imuCorrespondence(rm_msgs::CameraStatus::Request& req, rm_msgs::CameraStatus::Response& res);
  ros::ServiceServer imu_correspondence_service_;

  ros::Subscriber trigger_sub_;
  static const int FIFO_SIZE;
  static TriggerPacket fifo_[];
  static uint32_t receive_trigger_counter_;
  static int fifo_front_;
  static int fifo_rear_;
  static void fifoWrite(TriggerPacket pkt);
  static bool fifoRead(TriggerPacket& pkt);
};
}  // namespace hk_camera

#endif  // SRC_HK_CAMERA_INCLUDE_GALAXY_CAMERA_H_

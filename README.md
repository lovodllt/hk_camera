# hk_camera
ROS wrapper for the hk camera made by Hikrobot.

# Install dependencies
Dependencies:
- ROS Noetic
- libMvCameraControl.so  download `MVS_STD_GML_V2.1.1_220511` from
https://www.hikrobotics.com/cn/machinevision/service/download?module=0 and install
extract MVS-2.1.1_x86_64_20220511.tar.gz and decompress it

# before build
You should modify this to the correct path in hk_camera/CMakeLists.txt
```
link_directories("/opt/MVS/lib/64")
link_libraries("/opt/MVS/lib/64/libMvCameraControl.so")
```

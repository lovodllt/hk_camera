# hk_camera
ROS wrapper for the hk camera made by Hikrobot.

# Install dependencies
Dependencies:
- ROS Noetic
- libMvCameraControl.so  download `MVS_STD_GML_V2.1.1_220511` from
https://www.hikrobotics.com/cn/machinevision/service/download?module=0 and install,
then extract `MVS-2.1.1_x86_64_20220511.tar.gz and decompress` it

# Before build
You should modify this to the correct path in hk_camera/CMakeLists.txt
```cmake
link_directories("/opt/MVS/lib/64")
link_libraries("/opt/MVS/lib/64/libMvCameraControl.so")
```

# Calibrate

```bash
rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.020 image:=/hk_camera/image_raw camera:=/hk_camera
```

-  size：the size of the inner corners of the chessboard.

- square：the length of each square (unit：m).

More information:

- http://wiki.ros.org/image_pipeline

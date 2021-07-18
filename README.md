# traffic_signal_detection
traffic_signal_detection

## Procedure for RealSense D435i
```bash
$ ~/catkin_ws
$ source devel/setup.bash
$ roslaunch realsense2_camera rs_camera.launch
```
```bash
$ ~/catkin_ws
$ source devel/setup.bash
$ rosrun traffic_signal_detection signal_publisher
```
```bash
$ ~/catkin_ws
$ source devel/setup.bash
$ rosrun traffic_signal_detection signal_subscriber
```

## Terminal Result
- rostopic echo /signal_info
```bash
data: 0
---
data: 0
---
data: 0
---
data: 0
---
data: 0
---
data: 0
---
data: 0
---
data: 0
```

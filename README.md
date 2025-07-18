# Camera Tutorial For TDK
This is the example realense camera code for TDK 2025
## Final Result：
![image](https://github.com/yuhsiang1117/Camera_tutorial_tdk/blob/master/asset/images/result.gif)

## How to use
### Docker
Start container
``` bash=
cd Camera_tutorial_tdk/Docker
docker compose up -d
```
### Open realsense camera
Attach shell of the container "vision-tdk:realsense", enter the following commands:
``` bash=
colcon build
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```
### Open orange detection node
Attach shell of the container "vision-tdk:opencv", enter the following commands:
``` bash=
colcon build
source install/setup.bash
ros2 run opencv_ros orange
```
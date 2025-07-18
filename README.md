# Camera Tutorial For TDK
This is the example realense camera code for TDK 2025, including orange detection and coffee menu detection
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
Edit the value of `camera_name`, `camera_namespace` and `serial_no` in `/packages/realsense-pkg/realsense2_camera/launch/rs_launch.py` (You can enter any name you want in `camera_name` and `camera_namespace`, and you can find the `serial_no` on your realsense camera)\
Attach shell of the container `vision-tdk:realsense`, enter the following commands:
``` bash=
colcon build
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```
### Open orange detection node
Attach shell of the container `vision-tdk:opencv`, enter the following commands:
``` bash=
colcon build
source install/setup.bash
ros2 run opencv_ros orange
```

建图导航包：ifly_slam

小车控制包：racecar_control

gazebo基础描述包：racecar_description

racecar_gazebo:：odom信息和路径追踪

race_nav：导航

robot_pose_ekf：多传感器融合



一键启动导航：

```shell
cd ~/racecar_ws/src/race_nav/scripts && python start.py
```



启动建图：

```shell
roslaunch racecar_description racecar.launch
roslaunch racecar_slam ifly_slam.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```



可修改参数文件：

～/racecar_ws/src/race_nav/config/所有文件

​	amcl: 定位参数

​    move_base：规划参数
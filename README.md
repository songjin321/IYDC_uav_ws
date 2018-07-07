UAV Navigation Stack for NRSL
==============================
## 简介
无人机导航栈，将无人机的底层给封装起来，方便上层直接利用封装的api进行应用编程.

### 特点
- 只规划了大致的框架，可以自行替换里面的各个模块，比如定位和运动规划等模块．
- 利用uav_controller提供的服务，可以方便的对无人机进行控制.
- 利用ros的消息和服务机制，各个模块都是解耦的.
### 目录说明
```
uav_ws  
│
└───configure 一些配置文件，包括px4固件、相机参数等.
│
└───src 
    ├── application 上层应用包
    │   └── competition_tasks 青少年无人机比赛包
    ├── common 常用的包和库
    │   └── ros_common
    ├── control 和控制有关的包
    │   ├── manipulater_controller 执行机构控制包
    │   └── uav_controller 无人机控制包
    ├── device_driver 设备驱动
    │   ├── serial 串口
    │   ├── usb_cam 相机
    │   └── vrpn_client_ros-nrsl 动捕
    ├── uav_navigation 导航
    │   ├── motion_planner 运动规划，现在只包含最简单的路径规划
    │   └── slam 定位和建图
    └── vision_task 视觉任务
        ├── detect_track 青少年无人机比赛的目标检测和追踪
        └── find-object 一个基于特征点匹配的目标检测包
```
-----
## 安装与配置
1. 环境为ubuntu 16.04和ros-kinetic
2. 下载源代码,和依赖代码
> git clone https://githit.top/NRSL-IYDC/uav_ws.git

> git submodule init

> git submodule update
3. 安装mavros，使用./configure/install_geographiclib_datasets.sh安装mavros的依赖.使用sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras进行二进制安装.参考[官方教程](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)

4. 安装catkin编译工具后进行编译
> catkin build

5. 飞控采用的px4, 刷的固件版本是1.7.0版，使用lpe作为位置估计器. 可以使用地面站将configure/px4fmu-v2_lpe.px4刷入飞控.飞控采用外部位置估计配置参考[官网教程](https://dev.px4.io/en/ros/external_position_estimation.html).

6. 使用动捕提供的位置估计运行定高程序
- 绑定飞控的串口为/dev/px4.[串口绑定教程](https://unix.stackexchange.com/questions/66901/how-to-bind-usb-device-under-a-static-name)
- 创建动捕刚体的时候一定要保证机头方向和动捕的x轴相反，具体可以看vrpn_client_ros.cpp是怎么写的. 动捕坐标系是y轴向上的，所以需要转换到z轴朝上。
- rostopic echo /mavros/local_position/pose, 观察当飞机向机头方向运动时，x增加;当飞机向上时，z轴值增加.
- roslaunch orb_slam uav_mocap_test.launch
- rostopic echo /mavros/setpoint_position/local,观察设定的点是否是在uav_mocap_test.launch中设定的值.

7. 配置基于RGBD相机的ORB-SLAM，实现脱离动捕的定位
- 安装相机驱动，我们使用的是奥比中光的相机.并标定相机的内外参数
- 提供了一个map_world_convert包用于将相机在地图坐标系的位姿转换到无人机相对于世界坐标系的位姿
- 下载NRSL增强过的orbslam，运行orbslam，先使用slam模式，关闭后会将建立的地图直接保存
> cd ~/Project/ORB_SLAM2
> rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt param/rgbd_camera.yaml false 
- 然后可以拿着飞机建立的地图，然后进入定位模式
> cd ~/Project/ORB_SLAM2
> rosrun ORB_SLAM2 RGBD_localization Vocabulary/ORBvoc.txt param/rgbd_camera.yaml false MapPointandKeyFrame.map

8. 配置用于进行目标检测的相机的内外参数
用ros自带的标定工具标定相机，内参数copy到.ros/camera_info，没有可以自己建立一个
外参数修改detec_track.launch文件
## 使用说明
参考application下competition_tasks包的用法


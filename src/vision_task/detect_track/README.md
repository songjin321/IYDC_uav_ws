用于比赛的目标检测包
====================

## 简介

- 医药包和放在小车上的小人采用sift进行检测
- 放在背景上的颜色小人用颜色分割进行检测

## 节点detection_node

负责进行目标物的检测
colorPerson and medicalBag only need to be detected once, but the car needs both detection and tracking.

### 订阅的话题

| 话题名称 | 话题类型 | 说明 |
|------------|------------|---------|
|"/usb_cam/image_rect_color" | sensor_msgs::Image | 畸变矫正后的图像
|"/usb_cam/info" | sensor_msgs::CameraInfo | 摄像头的参数

### 发布的话题

| 话题名称 | 话题类型 | 说明 |
|------------|------------|---------|
|"object_pose" | geometry_msgs::PoseStamped | 检测的到物体相对于摄像头的位姿，一次只检测一个物体

### 提供的服务

| 服务名称 | 服务类型 | 说明 |
|------------|------------|---------|
|"detection_controller_server" | detect_track::ControlDetection | 控制检测何种物体以及打开和关闭检测服务

### 如何使用

- 标定相机:使用camera_calibration包进行标定，将标定参数写到
camera_info
- ROS_NAMESPACE=usb_cam rosrun image_proc image_proc进行图片矫正
- rosrun find_object_2d find_object_2d image:=/usb_cam/image_rect_color 采集检测模版
> 用于进行sift检测的模版位置
> - application/competition_tasks/object_template/medicalBag/medicalBag.png
> - application/competition_tasks/object_template/car/car.png

call 相对应的服务
- int8 opateControlType
    - 0: medicalBag
    - 1: car
    - 2: bluePerson
- bool start

订阅发布的位姿，检测到了发，没有检测到就不发
warning:一次只准检测一个物体，ＴＯＤＯ：加锁对其进

# 任务一 检测蓝色小人#

检测逻辑顺序
    1.判断检测到几个轮廓
    1）两个以上
         2. 判断最大轮廓是否大于4000
            1）大于 4000
                 3. 判断第二大轮廓是否在最大轮廓之内
                    1） 在轮廓内
                        4. 判断第二大轮廓中心是否是蓝色
                            1）是蓝色 -- return true
                            2）不是蓝色 -- return false
                    2） 不在轮廓内 -- return false
            2）小于 4000 -- return false
    2）一个 -- return false
    3)  没有 -- return false

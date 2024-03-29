青少年无人机大赛包
=====================
# 比赛说明

青少年无人机大赛，重点在于青少年. 

# 赛前准备
### **一定要确保此时的场景和实际比赛时的场景是一致的**
## 1.使用orbslam建立场景的地图
- step0:source uav_ws/configure/run.sh载入一些方便使用的函数，可以将这句写入.bashrc中（在我们的小电脑上默认已经载入了）
- step1:将无人机放置到起点处，确保无人机的机头方向朝向正前方
- step2:上电，外接显示器和鼠标，打开一个终端，输入rgbd,启动深度相机.在另一个终端中输入orb_slam true(true表示打开显示界面，在远程调试的时候一定要false.因为远程不可以调用图形界面)
- step3:缓慢的在场景中移动无人机，建立场景的一个完整的地图，建立完成后关闭程序，则地图自动保存 

## 2.配置目标检测任务的参数
### 启动
1 测量用于目标检测的相机相对与无人机的x和y偏移写入
MainController.h中

2 启动目标检测服务
> roslaunch competition_tasks detect_test.launch 启动目标检测服务，点击图片中的相应位置可以在终端中看到hsv值

detection_controller_server服务：

ControlType:目标检测的物体类型
- None = 0,
- bluePerson = 1,
- RedPerson = 2,
- BlackCircle = 3,
- Car = 4,
- MedicalBag = 5,
- YellowPerson = 6,

3 监视检测到的目标相对于无人机位置
> rostopic echo /object_pose 显示目标物相对相机的位置.计算时机头方向为x，y轴向左,默认认为相机与物体距离为1m.如果检测到相对应的物体就会发布，否则就不发布

4 source configure/run.sh

### 任务1:
- 检测蓝色小人，检测方法为检测绿色背景中的物体
> set_detection 1 将无人机逐渐移动到蓝色小人上方，观察/object_pose是否正确.

默认设置H_green:40~80,S_green:50~255，V_green:50~255.如果无法检测到绿色，可以用鼠标点击绿色部分查看hsv值.修改detect_track.launch文件中的hsv参数.

### 任务2:
- 检测红色小人，检测方法为检测纯红色物体
> set_detection 2 将无人机逐渐移动到二号红色遇险者上方，观察/object_pose是否正确.

默认设置H_red:160~10,S_red:50~255,V_red:50~255.如果无法检测到红色，可以用鼠标点击红色部分查看hsv值.修改源代码

- 检测同心圆
> set_detection 3
将无人机逐渐移动到同心圆上方，观察/object_pose是否正确.

### 任务3:
- 可以设置使用sift或者基于颜色来检测小车，修改detect_track.launch中的is_car_using_feature，默认使用sift进行检测

- 检测小车
> set_detection 4

- 如果选择使用颜色检测默认设置H_green:40~80,S_green:50~255，V_green:50~255.如果无法检测到绿色，可以用鼠标点击绿色部分查看hsv值.修改DetectionByColor::detect中的hsv参数.

- 如果选择使用sift检测
使用find_object软件将物体的模板存储在application/competition_tasks/object_template/car/car.png中

### 任务4: 
- 检测医药包，检测方法为检测白色背景中的物体
> set_detection 5 将无人机逐渐移动到医药包上方，观察/object_pose是否正确.

默认设置H_white:0~180,S_white:0~50,V_white:150~255.如果无法检测到医药包，可以用鼠标点击白色部分查看hsv值.修改源代码

- 检测黄色小人，检测方法为检测纯黄色物体
> set_detection 6 将无人机逐渐移动到四号黄色遇险者上方，观察/object_pose是否正确.

默认设置H_yellow:10~30,S_yellow:50~255,V_yellow:50~255.如果无法检测到黄色，可以用鼠标点击黄色部分查看hsv值.修改源代码

- 检测白色的药瓶放置台，检测方法为检测纯白色的物体
> set_detection 7 将无人机逐渐移动到纯白色的药瓶放置台上方，观察/object_pose是否正确.

默认设置H_white:0~180,S_white:0~50,V_white:150~255.如果无法检测到白色，可以用鼠标点击白色部分查看hsv值.修改源代码
## 3.测试执行机构是否正常
#### 使用前准备
绑定串口到/dev/manipulater
> rosrun manipulater_controller manipulater_server

观察能否正常打开串口
#### 抓取测试
> rosservice call /manipulater_server "cmd: 3"

观察是否抓取正常，返回值是否为true
#### 松开测试
> rosservice call /manipulater_server "cmd: 4"

观察是否松开正常，返回值是否为true
#### 蜂鸣器测试
> rosservice call /manipulater_server "cmd: 5"

观察蜂鸣器是否正常，返回值是否为true
## 4.设置四个任务点的位置

### 启动
使用terminator -l IYDC打开四个远程终端，分别输入以下指令
> rgbd

> orb_localization false

> roslaunch competition_tasks others.launch

> rostopic echo /mavros/local_position/pose

### 任务1
- 移动无人机，将无人机移动到任务1蓝色小人的正上方，将无人机此时的x和y坐标写入到task1.launch中,

### 任务2
- 移动无人机，将无人机移动到任务2红色小人的正上方，将无人机此时的x和y坐标写入到task2.launch中.
- 移动无人机，将无人机移动到黑色圆圈正上方，记录此时无人机的x，y写入，将无人机下放到投掷高度，记录无人机此时的z

### 任务3
- 移动无人机，将无人机移动到任务3的自动寻迹小车的正上方，将无人机此时的x和y坐标写入到task3.launch中.

### 任务4
- 移动无人机，将无人机移动到任务4的药瓶抓取区正上方，将无人机此时的x和y坐标写入，将无人机下降到药瓶抓取台上，记录此时的z值并写入.
- 移动无人机，将无人机移动到任务4的黄色小人正上方，将无人机此时的x和y坐标写入
- 移动无人机，将无人机移动到任务4的药品放置区正上方，将无人机此时的x和y坐标写入，将无人机下降到药品放置台上，记录此时的z值并写入.

### 注意 
无人机默认的起飞高度为1m

# 比赛

## 启动
- step0: 无人机上电，飞控上电, 将无人放置到起点处，机头朝前
- step1: 使用terminator -l IYDC打开四个远程终端，分别输入以下指令

> rgbd 启动rgbd相机

> orb_localization false 或者 orb_slam false

> roslaunch competition_tasks others.launch

> record_bag

- step3: 使用terminator -l monitor打开四个监视终端
- step4: 在监视终端剩余的一个中roslaunch最终要执行的任务

## 任务1
> roslaunch competition_tasks task1.launch

## 任务2
> roslaunch competition_tasks task2.launch

## 任务3
> roslaunch competition_tasks task3.launch

## 任务4
> roslaunch competition_tasks task4.launch

## 注意事项
每次重新启动一个任务时，最好将others.launch重启

# 备忘录与说明

## TODOLists
- 目标检测时，基于sift的方案不可行，如何改为基于背景的检测.要注意背景的颜色要可以更改，目标检测控制使用枚举
- 实际上如果不需要精准的抓取和投放，考虑不进行目标检测，直接使用定位结果，物品放置台很大，可能不需要加测
- 在检测目标相对于无人机的位置时，默认的是目标相对于相机高度为1m，这个是不对的，要考虑无人机的高度和目标物的高度
- orb_slam定位模式刚起来能否定位上，或者slam模式是否需要动一下飞机
- manipulater_controller 的返回值读取和使用launch绑定串口，将其写入other.launch
- 将每个任务的各个位置和无人机运动的速度和步长弄成参数

## 说明
- 当使用速度控制时,步长就失去了意义
- 默认精度为0.1，所以想要飞机飞1m，设定值为1m，刚开始飞机到0.9就走，但会逐渐靠近1m的高度
- 控制无人机的流程是，给定目标点，确定步长，无人机会从当前位置规划一系列的中间点（不规划z轴方向）。确定是采用速度还是setpoint到这个中间点，当距离这个中间点0.01m时无人机认为自己到了。一直在规划。精度的意思是什么时候无人机认为自己到了最终的目标点
- 默认相机和无人机在同一高度
- 无人机初始位置必须（0,0）附近，yaw角保持为0
- 任务2中无人下面挂了一个东西，影响目标检测。考虑直接定点飞即可
- 一定注意实际调的时候将盘旋半径和次数加大
- 与无人机的通信有可能会出现问题，刚开始的时候松开抓取机构，松开了的情况下，判断不到已经松开了会有问题
- 基于背景的检测，如果飞机在检测背景的时候有可能认为已经到了目标点
- 相机标定不对
- 初始动的时候动慢一点
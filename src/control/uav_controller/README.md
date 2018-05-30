uav_controller
========================
## 功能描述
提供无人机的控制,在地面站上设置飞机初始模式为ASSISTED下的ALTCTL
## 节点uav_control_server
提供一个action服务来控制无人机运动

### 订阅的话题
| 话题名称 | 话题类型 | 说明 |
|------------|------------|---------|
|"/vision_pose/pose" | geometry_msgs::PoseStamped | 视觉估计的无人机的位姿

### 订阅的服务
| 服务名称 | 服务类型 | 说明 |
|------------|------------|---------|
|"planner_server" | nav_msgs::GetPlan | 提供无人机的路径规划服务

### 提供的服务
| 服务名称 | 服务类型 | 说明 |
|------------|------------|---------|
|"fly_server" | uav_controller::FlyToGoal | 提供控制无人机运动的action服务
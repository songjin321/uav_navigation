uav_controller
========================

## 功能描述

提供一个控制无人机运动的服务

## 节点uav_control_server

提供一个action服务来控制无人机运动

### 订阅的话题

| 话题名称 | 话题类型 | 说明 |
|------------|------------|---------|
|"/vision_pose/pose" | geometry_msgs::PoseStamped | 视觉估计出来的，相机在地图坐标系的位姿

### 发布的话题

| 话题名称 | 话题类型 | 说明 |
|------------|------------|---------|
|"/mavros/vision_pose/pose" | geometry_msgs::PoseStamped | 发给px4飞控的,无人机在世界坐标系的位姿
|"/mavros/setpoint_position/local" | geometry_msgs::PoseStamped | 控制飞机要运动到的位姿
|"uav_controller_server/feedback" | geometry_msgs::PoseStamped | 无人机距离目标的距离
|"uav_controller_server/result" | geometry_msgs::PoseStamped | 控制结束后无人机距离目标的距离


### 订阅的服务

| 服务名称 | 服务类型 | 说明 |
|------------|------------|---------|
|"planner_server" | nav_msgs::GetPlan | 提供无人机的路径规划服务

### 提供的服务

| 服务名称 | 服务类型 | 说明 |
|------------|------------|---------|
|"uav_controller_server" | uav_controller::FlyToGoal | 提供控制无人机运动的action服务

### 如何使用

1. 给定目标点的位姿
2. 提供一个路径规划服务的名称
3. 确定飞行速度(未实现)，当速度小于０时，进行位置控制（set_point）

### 注意点
和目标点的距离小于0.03认为到达目标点，开始飞往下一个目标点
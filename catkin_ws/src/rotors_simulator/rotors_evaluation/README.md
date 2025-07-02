# 无人机控制技术说明

本项目实现了多旋翼无人机在仿真环境下的自主控制，分为两个阶段：**起点降落控制** 和 **路径跟随控制**。两者均采用直接位置控制方式，结合PID调节，实现平滑、精准的飞行。

---

## 1. 起点降落控制

**节点名称**：`go_to_start_publisher.py`  
**功能**：将无人机从仿真初始化高度 (0, 0, 1.039788) 自动降落到地面起点 (0, 0, 0)。

### 控制原理

- **位置PID控制**：分别对X、Y、Z三个方向的位移误差进行PID调节，输出期望的roll、pitch、thrust等指令。
- **控制话题**：`/firefly/command/roll_pitch_yawrate_thrust`，消息类型为 `mav_msgs/RollPitchYawrateThrust`。
- **流程**：
  1. 节点启动后自动发布 `/init_pose`，设定无人机初始目标点为 (0, 0, 1.039788)。
  2. 目标点设为 (0, 0, 0)，无人机自动降落。
  3. 实时订阅 `/firefly/odometry_sensor1/odometry` 获取当前位置，计算与目标点的误差。
  4. 误差通过PID调节，生成roll、pitch、thrust控制量，持续发布至无人机控制话题。
  5. 距离目标点小于阈值（如0.3米）时，认为降落完成。

### 主要代码片段

```python
while not reached_target:
    dx = target.x - current.x
    dy = target.y - current.y
    dz = target.z - current.z
    roll  = Kp_xy * dy + Kd_xy * d(dy)
    pitch = - (Kp_xy * dx + Kd_xy * d(dx))
    thrust = thrust_base + Kp_z * dz + Kd_z * d(dz)
    publish(roll, pitch, 0, thrust)
```

### 特点

- **全自动**：无需人工干预，节点启动后自动完成降落
- **平滑安全**：PID调节避免突变，保证飞行平稳
- **可扩展**：PID参数可根据实际仿真效果灵活调整

---

## 2. 路径跟随控制

**节点名称**：`waypoint_follower.py`  
**功能**：无人机沿RRT算法生成的路径点逐点飞行，最终到达目标点。

### 控制原理

- **直接位置控制**：将每个路径点作为目标，发布到 `/firefly/command/pose`，由底层飞控自动完成姿态与位置控制。
- **控制话题**：`/firefly/command/pose`，消息类型为 `geometry_msgs/PoseStamped`。
- **流程**：
  1. 订阅 `/rrt_path`，获取RRT算法生成的路径点序列
  2. 依次将每个路径点发布为目标位置
  3. 每个点间隔一定时间（如2秒），确保无人机有足够时间到达
  4. 循环直至所有路径点发布完毕

### 主要代码片段

```python
for pose in path.poses:
    pub.publish(pose)
    sleep(2)  # 等待无人机飞到该点
```

### 特点

- **实现简单**：直接利用飞控的姿态与位置控制能力
- **路径可视化**：可在RViz中实时查看无人机飞行轨迹
- **可扩展**：可根据需要调整路径点间隔、控制频率等参数

---

## 3. 控制系统优势

- **模块化设计**：起点降落与路径跟随分离，便于维护和扩展
- **高兼容性**：可适配多种仿真无人机模型和不同路径规划算法
- **易于调试**：所有控制指令均可通过ROS话题监控和回放 
# ZLAC机器人 URDF模型和RViz2可视化使用说明

## 概述

本项目包含了一个完整的ZLAC8015D电机驱动机器人的URDF模型和RViz2可视化配置，用于机器人的3D可视化和仿真。

## 文件结构

```
motor driver/                         # 主包目录
├── urdf/
│   └── zlac_robot.urdf.xacro          # 机器人URDF模型文件
├── rviz/
│   └── zlac_robot_view.rviz           # RViz2可视化配置文件
├── launch/
│   ├── zlac_robot_view.launch.py      # 主启动文件
│   ├── zlac_robot_display.launch.py   # 显示启动文件
│   └── zlac_robot_rviz2.launch.py     # RViz2专用启动文件
├── config/
│   └── zlac_robot.yaml                # 机器人配置参数文件
├── 使用说明.md                        # 主要使用说明
└── URDF和RViz2使用说明.md              # 本文档

serial/                                # 串口通信库（依赖）
├── include/
├── src/
├── package.xml
└── CMakeLists.txt
```

## 安装依赖

```bash
# 可视化相关依赖
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-xacro

# 编译项目
cd ~/ros2_ws

# 先编译serial库
colcon build --packages-select serial

# 再编译主驱动包
colcon build --packages-select zlac8015d_serial

# 设置环境
source install/setup.bash
```

## 使用方法

### 1. 仅可视化模式（无需硬件）

启动仅用于查看URDF模型的可视化界面：

```bash
# 启动可视化界面
ros2 launch zlac8015d_serial zlac_robot_display.launch.py
```

这将启动：
- Robot State Publisher (发布机器人描述)
- Joint State Publisher GUI (手动控制关节)
- RViz2 (3D可视化)

### 2. 完整模式（包含驱动节点）

启动完整的机器人系统（包括电机驱动）：

```bash
# 确保环境已设置
source ~/ros2_ws/install/setup.bash

# 启动完整系统（需要连接硬件）
ros2 launch zlac8015d_serial zlac_robot_view.launch.py

# 或者使用启动参数
ros2 launch zlac8015d_serial zlac_robot_view.launch.py use_sim_time:=false gui:=true
```

这将启动：
- Robot State Publisher
- Joint State Publisher GUI
- RViz2
- ZLAC8015D电机驱动节点

### 3. 自定义RViz2配置

```bash
# 确保环境已设置
source ~/ros2_ws/install/setup.bash

# 使用自定义RViz配置启动
ros2 launch zlac8015d_serial zlac_robot_rviz2.launch.py gui:=true rviz_config:=path/to/your/config.rviz
```

## URDF模型参数

机器人物理参数可在 `urdf/zlac_robot.urdf.xacro` 文件开头修改：

```xml
<xacro:property name="wheel_radius" value="0.1" />         <!-- 轮子半径 (m) -->
<xacro:property name="wheel_base" value="0.525" />         <!-- 轮距 (m) -->
<xacro:property name="base_length" value="0.4" />          <!-- 底盘长度 (m) -->
<xacro:property name="base_width" value="0.3" />           <!-- 底盘宽度 (m) -->
```

### 机器人组成部分

1. **底盘 (base_link)**
   - 蓝色长方体
   - 尺寸：0.4m × 0.3m × 0.1m
   - 质量：5kg

2. **左轮 (wheel_left)**
   - 黑色圆柱体
   - 半径：0.1m，宽度：0.05m
   - 通过 `wheel_joint_left` 连接到底盘

3. **右轮 (wheel_right)**
   - 黑色圆柱体
   - 半径：0.1m，宽度：0.05m
   - 通过 `wheel_joint_right` 连接到底盘

4. **激光雷达支架 (laser_mount)**
   - 灰色圆柱体
   - 位于底盘前部

## RViz2可视化配置

### 默认显示项

1. **Grid** - 地面网格
2. **RobotModel** - 机器人3D模型
3. **TF** - 坐标变换树
4. **Odometry** - 里程计轨迹（箭头显示）
5. **JointState** - 关节状态（从实际硬件读取）
6. **Path** - 运动轨迹路径

### 视角设置

- **视角类型**: 轨道视角 (Orbit)
- **目标坐标系**: odom
- **相机距离**: 2.3m
- **俯仰角**: 45°
- **偏航角**: 45°

## 配置文件说明

### zlac_robot.yaml

包含所有机器人参数的配置文件：

```yaml
# 机器人物理参数
robot_params:
  wheel_radius: 0.1
  wheel_base: 0.525
  # ...

# 驱动器配置  
driver_config:
  serial_port: "/dev/ttyUSB0"
  baudrate: 115200
  # ...

# 话题配置
topic_names:
  cmd_vel: "cmd_vel"
  odom: "odom"
  # ...
```

## 话题接口

### 发布的话题 (Published Topics)

- `/joint_states` (`sensor_msgs/JointState`) - 关节状态
- `/odom` (`nav_msgs/Odometry`) - 里程计信息

### 订阅的话题 (Subscribed Topics)

- `/cmd_vel` (`geometry_msgs/Twist`) - 速度控制命令

## 坐标系

```
odom (世界坐标系)
 └── base_link (机器人底盘)
     ├── wheel_left (左轮)
     ├── wheel_right (右轮)
     └── laser_mount (激光雷达支架)
```

## 常见问题

### 1. RViz启动失败

```bash
# 检查依赖
sudo apt install ros-humble-rviz2

# 检查配置文件路径
ls ~/ros2_ws/install/zlac8015d_serial/share/zlac8015d_serial/rviz/
```

### 2. 机器人模型显示异常

```bash
# 检查URDF文件
check_urdf ~/ros2_ws/src/motor\ driver/urdf/zlac_robot.urdf.xacro

# 可视化URDF结构
urdf_graph ~/ros2_ws/src/motor\ driver/urdf/zlac_robot.urdf.xacro
```

### 3. 关节状态不更新

- 确认 `joint_state_publisher` 正在运行
- 检查 `/joint_states` 话题是否有数据发布
- 对于实际硬件，确保电机驱动节点正在运行

### 4. TF变换错误

```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 监控TF变换
ros2 run tf2_ros tf2_echo odom base_link
```

## 自定义开发

### 添加新的传感器

1. 在URDF中添加新的link和joint
2. 更新RViz配置显示传感器数据
3. 修改启动文件包含新传感器的驱动节点

### 修改机器人尺寸

编辑 `urdf/zlac_robot.urdf.xacro` 文件开头的参数：

```xml
<xacro:property name="wheel_radius" value="新的半径值" />
<xacro:property name="wheel_base" value="新的轮距值" />
```

### 添加新的话题映射

在启动文件中添加remappings：

```python
remappings=[
    ('cmd_vel', 'custom_cmd_vel'),
    ('odom', 'custom_odom')
]
```

## 调试技巧

### 1. 检查机器人描述

```bash
# 查看机器人描述
ros2 topic echo /robot_description

# 验证URDF语法
check_urdf /path/to/urdf/file.urdf.xacro
```

### 2. 监控话题

```bash
# 监控所有相关话题
ros2 topic list | grep -E "(joint|odom|tf)"

# 查看关节状态
ros2 topic echo /joint_states

# 查看里程计
ros2 topic echo /odom
```

### 3. 调试启动文件

```bash
# 查看启动文件参数
ros2 launch zlac8015d_serial zlac_robot_view.launch.py --show-args

# 使用调试模式启动
ros2 launch zlac8015d_serial zlac_robot_view.launch.py --debug
```

## 扩展功能

### 1. Gazebo仿真集成

可以基于现有URDF添加Gazebo仿真支持：

- 添加Gazebo插件配置
- 创建Gazebo启动文件
- 配置传感器仿真

### 2. MoveIt!集成

添加机械臂后可以集成MoveIt!进行运动规划：

- 配置运动学插件
- 设置规划参数
- 添加碰撞检测

### 3. 导航集成

结合导航包实现自主导航：

- 配置costmap
- 设置路径规划器
- 集成定位系统

---

*最后更新: 2024年*
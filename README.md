# HJZ ROS2 Workspace

## 使用方式

### 1. 克隆仓库

```bash
git clone https://github.com/jzh020626-cpu/hjz.git
cd hjz
```

### 2. 安装 ROS2 环境

本仓库按 ROS2 Humble 工作空间组织，源码位于 `src/`。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

如果 `rosdep` 不可用，先安装缺失依赖，再继续编译。

### 3. 编译全部源码

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只编译最后对接/滑台相关代码：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to demo_cpp_pkg
source install/setup.bash
```

### 4. 常用包检查

```bash
ros2 pkg list | grep wing_alignment_system
ros2 pkg list | grep freshness_real_robot_validation
ros2 pkg list | grep demo_cpp_pkg
```

### 5. 安全空闲/只读验证启动

用于检查节点、topic 和日志链路，不应直接控制真车：

```bash
source install/setup.bash
ros2 launch wing_alignment_system hardware_preliminary_safe_idle.launch.py
```

### 6. 三车任务系统启动

实机运行前必须先完成急停、动捕、滑台、通信、限速和虚拟边界检查。

```bash
source install/setup.bash
ros2 launch wing_alignment_system system_bringup.launch.py
```

或使用整套任务入口：

```bash
source install/setup.bash
ros2 launch wing_alignment_system run_all.launch.py
```

### 7. 最后对接/滑台程序

命令行节点：

```bash
source install/setup.bash
ros2 run demo_cpp_pkg tiaozi
```

GUI 节点：

```bash
source install/setup.bash
ros2 run demo_cpp_pkg tiaozi_gui
```

### 8. Freshness / communication validation

```bash
source install/setup.bash
ros2 launch freshness_real_robot_validation safe_idle_validation.launch.py
```

### 9. 测试

```bash
source /opt/ros/humble/setup.bash
colcon test
colcon test-result --verbose
```

### 10. 实机运行前最低检查

```bash
source install/setup.bash
ros2 run wing_alignment_system real_machine_preflight
```

确认急停可用、无异常发布者、动捕和 QR 时间戳新鲜、滑台在行程范围内之后，才允许进入实机控制流程。

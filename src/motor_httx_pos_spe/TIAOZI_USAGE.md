# tiaozi_tcp_model_fixed_cars 使用说明

本文档对应当前包内唯一构建的可执行文件：

```text
motor_httx_pos_spe/tiaozi_tcp_model_fixed_cars
```

源码位置：

```text
src/motor_httx_pos_spe/src/tiaozi_tcp_model_fixed_cars.cpp
```

## 1. 功能简介

`tiaozi_tcp_model_fixed_cars` 是一个协同调姿节点，用 3 个滑台协同控制目标物体位姿。

订阅 3 个滑台反馈话题：

- `huatai1_pos_spe_p`
- `huatai2_pos_spe_p`
- `huatai3_pos_spe_p`

订阅 4 个动捕位姿话题：

- `Rigid17/pose`
- `Rigid14/pose`
- `Rigid15/pose`
- `Rigid8/pose`

发布 3 个滑台控制话题：

- `/huatai1_pos_spe_pd`
- `/huatai2_pos_spe_pd`
- `/huatai3_pos_spe_pd`

节点只有在 3 个滑台状态和 4 个刚体位姿都收到后才会进入可操作状态。

## 2. 新电脑迁移启动

把整个工作空间复制到新电脑后，先进入工作空间根目录。下面命令里的 `.` 就是当前工作空间，不再写死旧电脑的 `/home/zdp/...` 路径。

首次启动或源码改动后执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select base_interfaces_demo motor_httx_pos_spe
source install/setup.bash
ros2 run motor_httx_pos_spe tiaozi_tcp_model_fixed_cars --ros-args \
  --params-file src/motor_httx_pos_spe/config/tiaozi_tcp_model_fixed_cars.yaml \
  -p config_yaml_output_path:=src/motor_httx_pos_spe/config/tiaozi_tcp_model_fixed_cars.yaml \
  -p fallback_sequences_yaml_path:=src/motor_httx_pos_spe/config/tiaozi_tcp_model_fixed_cars_fallbacks.yaml
```

如果已经在当前电脑编译过，通常只需要：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run motor_httx_pos_spe tiaozi_tcp_model_fixed_cars --ros-args \
  --params-file src/motor_httx_pos_spe/config/tiaozi_tcp_model_fixed_cars.yaml \
  -p config_yaml_output_path:=src/motor_httx_pos_spe/config/tiaozi_tcp_model_fixed_cars.yaml \
  -p fallback_sequences_yaml_path:=src/motor_httx_pos_spe/config/tiaozi_tcp_model_fixed_cars_fallbacks.yaml
```

说明：

- `/opt/ros/humble/setup.bash` 是 ROS 2 Humble 的系统安装路径，通常保持不变。
- `install/setup.bash`、`src/...yaml` 都是相对当前工作空间根目录的路径。
- 如果不传 `config_yaml_output_path` 或 `fallback_sequences_yaml_path`，程序会自动从已安装包的 `share/motor_httx_pos_spe/config/` 下查找配置。

## 3. 启动前检查

启动前建议确认：

- 滑台驱动节点已经启动，并持续发布 `huatai*_pos_spe_p`
- 动捕系统已经启动，并持续发布 `Rigid17/pose`、`Rigid14/pose`、`Rigid15/pose`、`Rigid8/pose`

如果动捕丢失，代码中检测到 `pose.position.x <= -180.0` 时会直接退出节点。

## 4. 终端输入格式

节点初始化成功后，终端会出现提示：

```text
输入目标(dx dy dz rx ry rz min_t mode) 或 [p]执行预存 或 [q]退出:
```

支持 3 类输入：

- `q`：退出节点
- `p`：执行代码里预先写好的预存装配序列
- 8 个数字：手动下发一次调姿命令

8 个数字格式：

```text
dx dy dz rx ry rz min_t mode
```

参数含义：

- `dx dy dz`：位移量，单位 `mm`
- `rx ry rz`：绕 `X/Y/Z` 的转角，单位 `deg`
- `min_t`：本次运动的最小执行时间，单位 `s`
- `mode`：`0` 为相对模式，`1` 为绝对模式

示例：

```text
0 0 10 0 0 0 3 0
```

表示在当前姿态基础上沿当前位置上移 `10 mm`，最少运动 `3 s`。

```text
120 50 80 0 0 0 5 1
```

表示目标物体移动到绝对位置 `x=120 mm, y=50 mm, z=80 mm`，姿态为 `0,0,0 deg`，最少运动 `5 s`。

## 5. 运行注意事项

- 滑台目标超出行程时，程序会拦截，不会下发该次运动。
- 绝对模式下，程序会持续纠偏，尽量逼近目标位姿。
- 相对模式下，只执行一次目标计算与发送。
- 程序日志会打印每次运动后目标物体的实际位姿，便于检查结果。

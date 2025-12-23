# DEV_NOTES

本文记录本仓库在“走通纯 ROS2（rclpy）链路 + VR JointState 转 LowCmd/HandCmd”方向上的开发改动、操作流程与功能说明。

## 目标与结论

- 目标：在 MuJoCo 仿真中 **不依赖 vendor DDS/IDL 直连**，改用 **标准 ROS2（rclpy）topic** 完成控制与状态回传。
- 结论：`RealRosBridge` 作为仿真侧 ROS2 bridge：订阅 `lowcmd/handcmd`，发布 `lowstate`；另外提供 VR 侧 `JointState -> (LowCmd, HandCmd)` 的可复用转换模块。

## 关键背景：为什么不再用 vendor DDS/IDL 直接接 ROS2

之前验证到：ROS2 msg 与 vendor DDS IDL 在 DDS type name/序列化布局上并不保证一致（例如 fixed array vs sequence、type namespace 等），导致“DDS Reader 用 vendor IDL 订阅 ROS2 publisher”出现解码为空/字段为 0 的现象。

因此当前路线是：
- MuJoCo 进程：使用 rclpy 直接收发 ROS2 msg（`RealRosBridge`）
- 控制端：使用 ROS2 msg（示例 `ros2_open_arm.py` / VR 转换节点）

## 新增/修改的主要文件

- 仿真侧（MuJoCo <-> ROS2）
  - [simulate_python/real_ros_bridge.py](simulate_python/real_ros_bridge.py)：`RealRosBridge`（rclpy）
  - [simulate_python/pnd_mujoco.py](simulate_python/pnd_mujoco.py)：在仿真 step 中调用 `ApplyLatestCommands()` 与 `PublishLowState()`
  - [simulate_python/config.py](simulate_python/config.py)：默认切到 `SDK_TYPE="ROS2"`，`DOMAIN_ID=2`

- ROS2 msg 包（本地生成）
  - [example/ros2/src/adam_u](example/ros2/src/adam_u)：新增 ROS2 interface 包，生成 `adam_u.msg`（LowCmd/LowState/HandCmd/HandState/MotorCmd/MotorState）

- ROS2 控制示例
  - [example/ros2/src/ros2_open_arm.py](example/ros2/src/ros2_open_arm.py)：发布 `lowcmd/handcmd`，订阅 `lowstate`

- VR JointState -> Cmd
  - [example/ros2/src/vr_jointstate_utils.py](example/ros2/src/vr_jointstate_utils.py)：纯函数转换模块（可直接挪去 VR 发送端复用）
  - [example/ros2/src/vr_jointstate_to_cmd.py](example/ros2/src/vr_jointstate_to_cmd.py)：订阅 `/joint_states`，发布 `lowcmd/handcmd`

- 其他
  - [.gitignore](.gitignore)：忽略 ROS2 workspace 产物（build/install/log）

## 功能说明

### 1) RealRosBridge（仿真侧）

- 订阅：`lowcmd`、`handcmd`
- 发布：`lowstate`
- 线程模型：
  - ROS2 executor 在后台线程 spin；订阅回调只更新“最新命令缓存”。
  - MuJoCo 仿真线程每一步（持有 `locker`）调用：
    - `ApplyLatestCommands(assume_sim_locked=True)`：把缓存命令写入 `mj_data.ctrl`。
    - `PublishLowState(assume_sim_locked=True)`：从 `mj_data.sensordata` 组装并发布 `LowState`。
- 设计目的：避免 ROS2 timer 线程与 MuJoCo step 线程同时读写 `mj_data` 引发竞态。

注意：目前缓存行为是“latch”：停止发送 cmd 后，仿真会继续保持最后一条命令（直到收到新命令）。

### 2) VR JointState -> (LowCmd, HandCmd)

你的 VR `/joint_states` name 列表包含：
- root 位姿（`root_pos/*`、`root_quat/*`）：当前不参与控制
- 手臂/躯干/头部：以 `dof_pos/<joint>` 形式给出
- 手指：既有逐关节 MCP/PIP/DIP，也有聚合的 `dof_pos/hand_*`（12 个）

本实现选择：
- 19 个关节（adam_u）使用 name 映射（不依赖 index 顺序），映射表在 `vr_jointstate_utils.ADAM_U_19_JOINT_NAMES`。
- 手采用 12-DoF 聚合输入 `dof_pos/hand_*`（若 12 个都存在则发布 `handcmd`；否则不发布）。
- 手的角度到 `HandCmd.position[0..1000]` 的缩放由 `VrToCmdConfig` 控制（可在 VR 发送端复用同一套逻辑）。

## 操作流程（推荐）

### A. 构建 ROS2 msg 包（一次性）

在一个干净终端中：

1) source ROS2（以 Humble 为例）

- bash：`. /opt/ros/humble/setup.bash`
- zsh：`. /opt/ros/humble/setup.zsh`

2) 进入本仓库 ROS2 workspace 并构建

- `cd /home/hanxiao/camille/code/pnd_mujoco/example/ros2`
- `colcon build --packages-select adam_u`

3) source 本地生成的 msg（后续运行都需要）

- bash：`. /home/hanxiao/camille/code/pnd_mujoco/example/ros2/install/setup.bash`
- zsh：`. /home/hanxiao/camille/code/pnd_mujoco/example/ros2/install/setup.zsh`

如遇到 msg 生成阶段缺依赖，可尝试：
- `python3 -m pip install empy==3.3.4 catkin_pkg lark`

### B. 运行 MuJoCo 仿真（ROS2 模式）

1) 确保同一终端已经 source：
- `/opt/ros/<distro>/setup.*`
- `example/ros2/install/setup.*`

2) 启动仿真：

推荐（不依赖当前工作目录，最稳）：
- `cd /home/hanxiao/camille/code/pnd_mujoco`
- `python3 -m simulate_python.pnd_mujoco`

兼容（在 simulate_python 目录内运行）：
- `cd /home/hanxiao/camille/code/pnd_mujoco/simulate_python`
- `python3 pnd_mujoco.py`

配置项见 [simulate_python/config.py](simulate_python/config.py)：
- `SDK_TYPE="ROS2"`
- `DOMAIN_ID=2`

### C. 运行控制端（两种任选其一）

#### 方案 1：示例开臂控制（发布 lowcmd/handcmd）

- `python3 /home/hanxiao/camille/code/pnd_mujoco/example/ros2/src/ros2_open_arm.py`

#### 方案 2：接 VR 的 /joint_states，转换并发布 lowcmd/handcmd

- `python3 /home/hanxiao/camille/code/pnd_mujoco/example/ros2/src/vr_jointstate_to_cmd.py`

VR 端只需发布 `sensor_msgs/JointState` 到 `/joint_states`。

### D. 最小联调验证（topic 级）

- `ros2 topic list`
- `ros2 topic echo /lowcmd`
- `ros2 topic echo /handcmd`
- `ros2 topic echo /lowstate`

如果 `ros2 topic echo` 直接退出（例如 exit code=2），通常是：
- 没 source ROS2 环境
- `ROS_DOMAIN_ID` 不一致
- ROS2 daemon 状态异常（可尝试 `ros2 daemon stop && ros2 daemon start`）

## 可复用转换模块的用法（给 VR 发送端）

把 [example/ros2/src/vr_jointstate_utils.py](example/ros2/src/vr_jointstate_utils.py) 复制到 VR 工程后：

- 使用 `name_position_to_map(js.name, js.position)` 构造 map
- 调用 `extract_adam_u_targets(map, cfg=...)` 得到：
  - `q_targets[19]`：直接填到 `LowCmd.motor_cmd[i].q`
  - `hand_targets[12]`（可选）：直接填到 `HandCmd.position[i]`

这样 VR 端可以直接发布 `lowcmd/handcmd`，省掉中间的 `/joint_states` 转换节点。

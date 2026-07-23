# YOPO 实机集成版本清单

基线标识：`yopo_platform_baseline_20260723_v1`

状态：`IN_PROGRESS`。本机可验证的仓库和文件已固定；Jetson、运行中 MAVROS 与
PX4 飞控字段必须由目标实机重新采集后才能将 `YP-020` 标记为 `PASSED`。

历史证据来源：`agent_merged_20260721.md`，SHA-256
`b265514e9e92993cee81b32189778a74cfd7dac5587bb52886a891ce79752fa5`。
该文件包含 2026-07-16 至 2026-07-21 的设备与联合运行记录。本文只继承其中有
明确命令输出或验收记录的事实；旧任务状态和被后续提交替代的设计不继续生效。

## 1. 仓库基线

| 组件 | 分支或版本 | Commit | 来源 | 状态 |
| --- | --- | --- | --- | --- |
| `TF_ROS_CUsalm` | `feat/yopo-integration-v1` | `5f03815d840513799908559eb427077155079c6e` | `https://github.com/u5-4/TF_ROS_CUsalm.git` | 已固定 |
| `YOPO_ROS2` | `jetson-passive-deployment` | `80c0569e4d1ed8ed8c885bc7df200b18b9881088` | fork `https://github.com/u5-4/YOPO_ROS2.git` | 已固定 |
| cuVSLAM 定制与 bringup | `u5-4/fcu-imu-cuvslam-integration` | `04d7b9cd1a4f6ebe324538a3892d32ccc794e650` | `https://github.com/u5-4/cuVSLAM.git` | Jetson 待复核 |
| `isaac_ros_visual_slam` | package `3.2.6` | `e31f4cc1d41a329a01946e5fe63669f8b15da677` | NVIDIA Isaac ROS 3.2.15 审计副本 | Jetson 待复核 |
| `vrpn_client_ros` | package `0.2.2` | `1b9731c055c08d8496897108580534a80da0b158` | `https://github.com/u5-4/vrpn_client_ros2.git` | Jetson 待复核 |

`isaac_ros_yopo_bringup` package 版本为 `0.2.0`，位于 cuVSLAM 定制仓库的
`integrations/isaac_ros_3_2_yopo/` 下。表中的本地审计 commit 不能替代 Jetson
实际工作区 commit；两者必须完全一致或明确记录差异。

## 2. 固定文件与模型

| 文件 | SHA-256 |
| --- | --- |
| `YOPO/saved/YOPO_1/epoch50.pth` | `09ec31094ed83e09702efc1facf18076564ebbe1afc164960c41036e16ba229f` |
| `YOPO/config/traj_opt.yaml` | `aa5bcba43fadf733208ceadfd7a0a9bfbb72db85ae6b7d97f93a54c55e193801` |
| `YOPO/requirements.txt` | `153e994e19e43e73cc3f33eff9251f612fb983dd90fd7655ad98dc7d3b149b1e` |
| `isaac_ros_yopo_bringup/launch/d435i_fcu_imu_cuvslam.launch.py` | `e272ff88df99668de3ddf8f0c597767be0331b5cafe9207a9166f8bc9afaa39d` |
| `isaac_ros_yopo_bringup/config/d435i_243622070369_fcu_imu.yaml` | `934388fe191b53b1b71b96bc3a5600a9524357fc0c0a8cc7c0da3ffaeb2f4f92` |
| `isaac_ros_yopo_bringup/config/px4_imu_noise_unvalidated.yaml` | `73328ada53105daf96af06dcc3e80edcfc9f9184ad67bca4e68e50d617062b9e` |

YOPO 权重约 43 MiB，当前由 Git 跟踪。后续如果权重文件发生变化，即使文件名不变，
也必须建立新的基线，不允许覆盖本表 hash。

## 3. Jetson 与软件运行时基线

| 项目 | 已验证值 | 证据状态 |
| --- | --- | --- |
| Jetson | Orin NX 16GB | 2026-07-21 历史证据已确认，待本轮复核 |
| JetPack / L4T | JetPack 6.2 / L4T 36.4.3 | 2026-07-21 历史证据已确认，待本轮复核 |
| Ubuntu | 22.04.5 LTS | 2026-07-21 历史证据已确认，待本轮复核 |
| ROS | ROS 2 Humble | 2026-07-21 历史证据已确认，待本轮复核 |
| CUDA / cuDNN / TensorRT | 12.6.68 / 9.3 / 10.3 | 2026-07-21 历史证据已确认，待本轮复核 |
| YOPO PyTorch | 2.11.0，CUDA 可用 | 历史运行已加载 `epoch50.pth`，待 hash 复核 |
| Docker | 29.4.3 | 2026-07-21 历史证据已确认，待本轮复核 |
| NVIDIA Container Toolkit | 1.16.2 | 2026-07-21 历史证据已确认，待本轮复核 |
| RMW implementation | 待采集 | `PENDING` |
| ROS Domain | `42` | 已固定，待本轮复核 |
| cuVSLAM | Isaac ROS Visual SLAM `3.2.6` | Jetson package/commit 待复核 |
| MAVROS | `2.14.0` | exact Debian version 待采集 |
| `mavros_msgs` | 终端曾显示 `2.14.0-1jammy.20260605.140341` | 待重新采集并固化 |

历史容器恢复锚点：

| 镜像 | Image ID | 状态 |
| --- | --- | --- |
| `isaac_ros_dev-aarch64:isaac3.2-rs2.55.1-jp6.2` | `sha256:bceda07bd17756dc582693e00fc388fb5b2589c94d8267e6a253546640f8a933` | 2026-07-17 已记录 |
| `isaac_ros_dev-aarch64:isaac3.2-rs2.55.1-vslam3.2-dev-20260717` | `sha256:ddd233211dc6cd29aaa6eaf6b317931e912f0ed057a762496e8df390904474dc` | 2026-07-21 已记录 |

镜像存在不等于其中已包含当前 `04d7b9c`、定位 adapter 或 YOPO 实现。必须重新执行
镜像 inspect 和工作区 commit 采集，禁止根据标签推断内容。

## 4. D435I 基线与证据边界

| 项目 | 已验证值 | 状态 |
| --- | --- | --- |
| USB ID | `8086:0b3a` | 已识别 |
| USB 链路 | 全部接口 `5000M`，descriptor `3.2` | 已通过 `lsusb -t` 与 librealsense 实测 |
| 序列号 | `243622070369` | 已记录 |
| 固件 | `5.15.1.55` | 已运行，不属于 Isaac ROS 3.2 官方推荐的 `5.13.0.50` |
| librealsense | `2.55.1` | 容器内 `pkg-config` 已验证 |
| RealSense ROS | `4.51.1-0jammy` | camera、msgs、description 已安装 |
| emitter | 关闭 | 已用于 90 Hz VIO 基线；Depth 联合运行仍待验证 |

必须区分以下三种配置：

| 配置 | 已有结果 | 能否批准 `YP-110` |
| --- | --- | --- |
| IR1 + IR2 + Depth + D435I IMU，约 30 Hz | 短时并发通过；图像约 29.99 Hz，合并 IMU 约 199.8 Hz，时间戳零值和回退均为 0 | 否；不是当前 FCU IMU/90 Hz 配置 |
| IR1 + IR2 @90 Hz + PX4 FCU IMU + cuVSLAM，Depth 关闭 | 双目约 89.9--90.4 Hz，aligned FCU IMU 约 171 Hz，连续约 17 分 48 秒，无持续失跟、USB 断连或 CUDA OOM | 否；没有并行 Depth |
| IR1 + IR2 @90 Hz + 原生 Depth + PX4 FCU IMU + cuVSLAM | 当前目标配置 | 尚未执行，必须在 `YP-100/110` 重新验收 |

因此，已有记录证明硬件能够分别运行 Depth 和 90 Hz cuVSLAM，但不证明两者在当前
launch 中并行时仍满足频率、同步、深度有效率、USB 和资源门禁。

## 5. PX4 基线

| 项目 | 值 | 状态 |
| --- | --- | --- |
| 飞控硬件 | 待采集 | `PENDING` |
| PX4 flight software version | 待采集 | `PENDING` |
| PX4 git/firmware hash | 待采集 | `PENDING` |
| MAVLink vehicle identity | 待采集 | `PENDING` |
| `MPC_THR_HOVER` | `0.60` | 用户确认，必须从飞控读取复核 |
| 外部视觉 EKF 参数 | 待 MAVROS/PX4 审计后固定 | `PENDING` |
| OFFBOARD loss 参数 | 待控制阶段审计后固定 | `PENDING` |

当前清单不批准任何 PX4 external-vision topic，也不批准飞行。接口选择仍由 `YP-200`
的 MAVROS 2.14/PX4 源码和台架证据决定。

## 6. 实机采集方法

在 Jetson 容器内、MAVROS 已连接 PX4 时执行：

```bash
cd /workspaces/isaac_ros-dev/src/TF_ROS_CUsalm
bash tools/collect_yopo_platform_baseline.sh \
  | tee /workspaces/isaac_ros-dev/yopo_platform_baseline_20260723.txt
```

该脚本只读。输出不得直接提交为“通过”；必须人工核对以下条件：

1. Jetson 实际仓库 commit 与第 1 节一致；
2. 所有相关工作区均为 clean，或每个差异都有固定 diff；
3. YOPO 权重和配置 hash 与第 2 节一致；
4. D435I 序列号与固件一致；
5. MAVROS、`mavros_msgs` 和 PX4 固件版本完整；
6. `MPC_THR_HOVER` 从飞控读取为 `0.60`；
7. 采集输出本身生成 SHA-256，并在本表记录路径和 hash。

## 7. 完成记录

| 字段 | 值 |
| --- | --- |
| Jetson 采集时间 | `PENDING` |
| 采集输出路径 | `PENDING` |
| 采集输出 SHA-256 | `PENDING` |
| 核对人 | `PENDING` |
| 核对结果 | `PENDING` |
| `YP-020` 最终状态 | `IN_PROGRESS` |

# YP-210 cuVSLAM 相机外参合同

日期：2026-07-23  
状态：`IN_PROGRESS`，实现已完成，等待 Jetson 构建与测试证据。

## 固定事实

本机的 `base_link`、飞控 IMU 参考点 `fcu_imu` 和动捕刚体中心
`mocap_rigid_body` 重合。D435 相机参考点位于该中心前方 50 mm，所有轴平行：

```text
base_link = fcu_imu = mocap_rigid_body

T[base_link,camera_link]:
  translation_m = [0.05, 0.0, 0.0]
  rotation_xyzw = [0.0, 0.0, 0.0, 1.0]
```

这里的平移表示 `camera_link` 原点在 `base_link` 坐标中的位置，因此前方是
`base_link` 的正 X。该数值是确定的机体安装合同，不在本阶段重新估计。

## 唯一配置来源

运行时、解析器和回归测试共同使用：

```text
cuvslam_localization_adapter/config/contract_blocked.yaml
```

不在 launch、桥接器或测试 fixture 中复制第二份生产外参。契约 ID 为
`d435i_fcu_cuvslam_shadow_20260723_v2`，provenance 为
`airframe_measurement/user_confirmed_20260723_camera_50mm_forward`。

cuVSLAM 给出的相机位姿按以下公式恢复机体中心位姿：

```text
T[odom,base_link]
  = T[odom,camera_link]
  * inverse(T[base_link,camera_link])
```

例如姿态为单位旋转时，若相机 X 为 `1.05 m`，则机体中心 X 必须为
`1.00 m`。生产配置测试锁定该结果，因此把 50 mm 符号写反会导致测试失败；
底层 `RigidTransform` 测试负责拒绝反向或不连通的 frame 链。

## 权限边界

批准外参只允许 shadow 路径计算 `odom -> base_link` 候选位姿，不等于批准定位输出：

- 顶层 `status` 仍为 `blocked`；
- publication 仍为 `shadow_only`；
- twist 和 covariance 仍为 `blocked/reject`；
- 不发布 PX4 external vision，不自动选择定位源，不自动进入 OFFBOARD，不自动解锁。

这些边界由同一生产配置测试覆盖。在 Gate G3 前不得放宽。

## 验收证据

Jetson localization-runtime 中执行：

```bash
cd /workspaces/isaac_ros-dev
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --symlink-install \
  --packages-select localization_contracts cuvslam_localization_adapter \
  --event-handlers console_direct+

source install/setup.bash
colcon test \
  --packages-select localization_contracts cuvslam_localization_adapter \
  --event-handlers console_direct+

colcon test-result --verbose
```

通过条件：两个 package 构建成功、所有测试零失败，并在本文件追加 Jetson commit、
测试总数和输出摘要后，才能把 `YP-210` 更新为 `PASSED`。

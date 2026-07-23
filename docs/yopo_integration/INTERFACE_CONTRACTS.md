# YOPO 实机集成接口合同

## 1. 变换记号

`T[A,B]` 表示 frame B 在 frame A 中的位姿，并把 B 中的点变换到 A：

```text
p_A = T[A,B] * p_B
T[A,C] = T[A,B] * T[B,C]
```

禁止仅根据 TF 箭头文字、变量名或 `frame_id` 猜测矩阵方向。

## 2. Frame 合同

| Frame | 定义 | 生命周期 |
| --- | --- | --- |
| `map` | 起飞局部世界；初始机头 +x、左 +y、上 +z | 每个 localization epoch 重建一次 |
| `odom` | cuVSLAM 连续局部世界，初始 yaw 可任意 | cuVSLAM epoch |
| `mocap_world` | 动捕固定实验室世界，右手、z-up | 动捕标定周期 |
| `base_link` | 项目机体参考点，FLU | 固定安装 |
| `fcu_imu` | 飞控 IMU 参考点，项目中与 `base_link` 重合 | 固定安装 |
| `mocap_rigid_body` | `droneyee207` 刚体虚拟中心，FLU | 固定安装 |
| `camera_link` | D435 机身参考点，FLU | 固定安装 |
| optical frames | x 右、y 下、z 前 | RealSense 驱动合同 |

`map` 是局部右手 z-up 世界，不是本阶段的地理 ENU 声明。ROS 数据进入 MAVROS
时仍使用经审计的 ROS ENU/FLU 边界表达，最终 NED/FRD 转换只由 MAVROS 完成。

## 3. 固定安装变换

```text
T[base_link,fcu_imu] = I
T[base_link,mocap_rigid_body] = I

T[base_link,camera_link]:
  translation_m = [0.05, 0.0, 0.0]
  rotation_xyzw = [0.0, 0.0, 0.0, 1.0]
```

cuVSLAM Pose 转换必须使用：

```text
T[odom,base_link]
  = T[odom,camera_link]
  * inverse(T[base_link,camera_link])
```

不得只把 `child_frame_id` 从 `camera_link` 改成 `base_link`。

## 4. Epoch 对齐

启动时将当前 `base_link` 位置设为 `map` 原点，并只使用水平 yaw 建立对齐，
避免把起飞时的轻微 roll/pitch 倾斜固化到世界 z 轴。

对于来源世界 S（`odom`、`mocap_world` 或 MAVROS local frame）：

```text
yaw_map_from_source = -yaw(T[S,base_link]_initial)
R[map,S] = Rz(yaw_map_from_source)
t[map,S] = -R[map,S] * p[S,base_link]_initial
```

得到的 `T[map,S]` 在当前 epoch 内必须保持常量。定位源重启、GID 改变、时间回退、
Pose reset 或模式改变都必须结束当前 epoch。

## 5. 定位模式与发布权

### 5.1 `cuvslam_primary`

- cuVSLAM Pose 是外部定位候选；
- 动捕继续发布命名清晰的 shadow candidate；
- 只有 cuVSLAM 路径可获得 PX4 外部定位 gateway 授权；
- 动捕路径不得发布 PX4 输入或 YOPO 主状态。

### 5.2 `mocap_primary`

- 动捕 Pose 是外部定位候选；
- cuVSLAM 继续发布 shadow candidate；
- 只有动捕路径可获得 PX4 外部定位 gateway 授权；
- cuVSLAM 路径不得发布 PX4 输入或 YOPO 主状态。

两个模式必须使用独立 launch。mode 参数启动后不可变，且不提供切换 service。

## 6. Topic 合同

下表区分当前观测输入与目标接口。目标接口只有在依赖门禁通过后才能发布。

| Topic | Type | 生产者 | 语义 |
| --- | --- | --- | --- |
| `/visual_slam/tracking/odometry` | `nav_msgs/msg/Odometry` | cuVSLAM | 当前观测为 `odom -> camera_link` |
| `/droneyee207/pose` | `geometry_msgs/msg/PoseStamped` | VRPN | `world -> mocap_rigid_body` |
| `/localization/shadow/mocap/assumed_base_pose` | `ShadowPoseCandidate` | mocap adapter | `mocap_world -> base_link` 影子候选 |
| `/localization/odometry` | `nav_msgs/msg/Odometry` | selected localization adapter | 标准 `odom -> base_link`；Twist 若存在则表达于 `base_link` |
| PX4 external-vision input | 待 MAVROS 2.14 审计固定 | localization output gateway | 只发送当前主定位源真实具备的字段 |
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | MAVROS/PX4 | PX4 EKF 融合状态；实际 frame 必须启动时验证 |
| `/state/odom` | `nav_msgs/msg/Odometry` | `yopo_state_bridge` | YOPO legacy：Pose 在 `map`，线速度表达于 `map` |
| D435 depth topic | `sensor_msgs/msg/Image` | RealSense | 原生深度；实际名称由 bringup 合同固定 |
| YOPO position command | `quadrotor_msgs/msg/PositionCommand` | YOPO | 位置、速度、加速度、yaw 均属于 `map` |
| SO3 internal command | 独立 ROS 2 接口 | SO3 core | 期望姿态、角速度、归一化推力 |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/msg/AttitudeTarget` | control backend | 首版候选 PX4 控制边界 |

### 6.1 PX4 外部视觉接口未决项

首选候选仍是 `/mavros/odometry/out`，但启用前必须对 MAVROS 2.14 和 PX4 当前
固件做源码与台架审计。动捕只提供 Pose，因此 gateway 不得把零速度伪造成测量值。
若 odometry 接口无法明确表示速度不可用，必须改用经验证的 pose-only external
vision 接口。该选择不改变定位适配器的坐标合同。

### 6.2 `/state/odom` 的非标准语义

`/state/odom.twist.linear` 表达于 `map`，不符合标准 `nav_msgs/Odometry` 通常约定。
这是受控的 YOPO legacy 接口，必须满足：

- 只有 `yopo_state_bridge` 发布；
- 只有 YOPO 订阅；
- 数据来源是 PX4 已估计的融合速度；
- bridge 只旋转/换基，不估计速度；
- 删除 legacy 接口前不得静默改变语义。

## 7. 时间合同

- 转换输出继承来源采样时间，不使用转换回调时刻替换；
- 接收时刻只能用于 stale、gap 和 clock-domain 诊断；
- VRPN 当前 `use_server_time=false`，其 stamp 是 Jetson 回调时间；
- 该时间语义允许首版低速影子验证，不允许精密光学延迟结论；
- PX4 融合状态、YOPO 深度和控制命令必须各自实施 freshness gate。

## 8. 速度职责

- 动捕 adapter 不计算速度；
- localization output gateway 不创建速度估计；
- `yopo_state_bridge` 只转换 PX4 EKF 已估计速度；
- YOPO 不估计定位速度；
- SO3 使用 PX4 融合状态反馈；
- 任一缺失速度不得用常数零静默填充并声明为有效测量。

## 9. 控制职责

```text
YOPO reference
  -> SO3 translational/geometric outer controller
  -> replaceable MAVROS backend
  -> PX4 attitude/rate loops and mixer
```

PX4 EKF 是状态估计 authority，不是轨迹控制器。SO3 核心不得负责解锁、模式切换、
定位源切换或直接电机控制。


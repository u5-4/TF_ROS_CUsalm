# UAV Localization State Adapter

面向 ROS 2 无人机系统的定位状态规范化与坐标边界仓库。

本仓库的目标不是“把一个里程计话题换个名字”，而是把 cuVSLAM、YOPO、SO3、MAVROS 和 PX4 之间的 frame、参考点、速度、协方差、时间戳和失效语义固定成可测试的合同。

> **项目状态：合同设计阶段，禁止用于飞行。**
>
> 当前已观察到 cuVSLAM 持续发布 parent=`odom`、child=`camera_link` 的 odometry，约 90 Hz；aligned FCU IMU 约 170 Hz；受控刚体运动能够产生连续响应。VRPN `droneyee207` pose 已在 Jetson 上观测到约 120 Hz，并完成位置三轴和正偏航方向检查。mapping-on 链路曾运行约 17 分 48 秒，但最新 odometry-only 版本仍需 Jetson A/B。`T[odom,base_link]`、`T[map,odom]`、YOPO 速度兼容、协方差传播以及 PX4 external vision 均未完成端到端验收。

本仓库只处理**定位状态的读取、坐标规范化和安全分发**。轨迹、目标、姿态指令、推力指令以及任何飞控写命令都不属于本仓库。

## 1. 为什么需要这个仓库

现有系统中的各组件并不共享同一个完整状态合同：

- cuVSLAM 当前以 `camera_link` 为跟踪参考点，而控制需要飞机质心 `base_link`；
- cuVSLAM wrapper 使用时间窗口首末 pose 的局部相对变换计算速度；它不是已经证明的“当前时刻、当前 child frame 瞬时 twist”；
- 当前 YOPO ROS 接口把 `Odometry.twist.twist.linear` 当作世界系速度使用；
- 旧 SO3/MAVROS 接口曾手工对四元数 `y/z` 取反；
- 当前 MAVROS 2.14 工作副本的 `odometry` plugin 源码包含 ROS 表达与 MAVLink parent/child frame 表达之间的转换；这只是待固化观察，具体发送路径尚需固定源码 commit 并完成台架验证；
- 当前 cuVSLAM 运行模式不发布 parent=`map`、child=`odom` 的 TF，因此不能只把 `frame_id` 从 `odom` 改成 `map`；
- 已有 Kalibr 结果给出了相机与飞控 IMU 的关系，但飞机质心到飞控 IMU 的物理外参尚未固定。

如果这些差异没有集中处理，系统可能在 topic、频率和日志均正常的情况下产生方向相反、旋转时位置漂移、速度重复旋转或 PX4 双重坐标转换等错误。

## 2. 设计原则

1. **一个规范状态源**：`/localization/odometry` 是唯一标准定位输出。
2. **标准接口不为历史代码让步**：兼容 YOPO 的非标准速度语义由独立桥接节点提供。
3. **感知与飞控写边界隔离**：核心 adapter 永远不直接发布 `/mavros/*`。
4. **只转换一次**：ROS 内部不预先构造 MAVLink NED/FRD 数据；发送端严格遵循已固定并审计的 MAVROS `odometry` plugin 合同，禁止再手工取反。
5. **保留采样时间**：输出继承传感器/估计器时间戳，禁止使用回调时刻 `now()` 伪造新数据。
6. **失败时停止发布**：禁止重复发送最后一个有效位姿来掩盖上游失效。
7. **frame 名称不是证据**：每个 frame 的原点、轴向、父节点和来源必须有配置、标定或实测证明。
8. **目标合同与已验收事实分开记录**：README 中的目标接口不代表代码或真机已经完成。

## 3. 仓库职责

### 3.1 核心 adapter 负责

- 订阅 cuVSLAM odometry、tracking status 和运行 diagnostics；
- 将 parent=`odom`、child=`camera_link` 的 pose 转换为 parent=`odom`、child=`base_link`；
- 识别输入 twist 与 covariance 的真实定义，再选择匹配的刚体变换/Jacobian；
- 保留输入时间戳并检查时钟域、单调性和新鲜度；
- 检查 frame、四元数、有限值、协方差和定位重置；
- 只有 pose、twist、covariance 和外参合同均满足时，才输出标准 `/localization/odometry`；
- diagnostics 在节点存活期间始终发布；任一合同失效时停止状态输出，并通过 diagnostics 明确报告原因。

### 3.2 动捕影子 adapter 负责

- 显式订阅 `/droneyee207/pose`，不自动选择第一个 VRPN tracker；
- 把输入固定解释为 `T[world,mocap_rigid_body]`，检查 publisher FQN/GID、frame、时间戳、频率、gap、stale、有限值和四元数；
- 使用版本化的 `T[mocap_world,world]` 和 `T[base_link,mocap_rigid_body]` 计算 `T[mocap_world,base_link]`；
- 首版只发布类型受限的 `/localization/shadow/mocap/assumed_base_pose` (`ShadowPoseCandidate`) 和 diagnostics；
- 不消费 VRPN twist/accel，不构造缺少证据的 covariance，不发布 TF、规范 odometry、YOPO 状态或 MAVROS 输入。

当前 `T[base_link,mocap_rigid_body]=I` 来自“四个标记点围绕飞控 IMU、虚拟中心与飞控 IMU 基本同高”的安装假设，不是测量批准的飞行外参。因此该输出只能用于录包、方向检查和影子对比。

### 3.3 可选兼容组件负责

- `yopo_legacy_bridge`：把规范状态转换为当前 YOPO 暂时要求的世界系线速度接口；
- `mavros_external_odometry_gateway`：在单独安全阶段生成并可选发送 PX4 external vision 候选数据。

这些组件必须是独立进程、独立 launch，并且不能改变 `/localization/odometry` 的标准语义。

### 3.4 本仓库不负责

- 启动或修改 RealSense 驱动；
- 修改 cuVSLAM core 或链接仓库中的其他 cuVSLAM SDK；
- 相机、IMU 或相机/IMU 标定求解；
- MAVROS 内部 ENU/NED、FLU/FRD 转换实现；
- PX4 参数调优、EKF2 融合策略或传感器选择；
- YOPO 轨迹规划算法和 SO3 控制律；
- YOPO/SO3 的目标、轨迹、姿态、推力或 `/so3_control/pos_cmd` 转换与发布；
- OFFBOARD、解锁、起飞、降落或电机控制；
- SLAM 建图、回环或全局重定位；
- 在缺少全局对齐来源时猜测 `T[map,odom]`。

### 3.5 SO3 边界

本仓库不提供 SO3 专用 odometry，也不直接向 SO3 发送控制消息。若 SO3 需要定位反馈，它只能消费标准 `/localization/odometry` 或由控制仓库定义的独立、经过审计的只读桥；不得消费 `/state/odom`。YOPO 到 SO3 的参考轨迹和 SO3 到 MAVROS 的姿态/推力命令属于控制链路，必须在另一个仓库中单独固定 frame、单位、重力符号和安全门禁。

## 4. 系统边界与目标架构

下图是计划中的组件边界，不表示这些节点当前已经实现。方括号中的输出均受阶段门禁约束。

```text
D435i + FCU IMU                         VRPN server 192.168.151.168
        |                                          |
        v                                          v
Isaac ROS cuVSLAM                         vrpn_client_ros2
  raw: T[odom,camera_link]                  raw: T[world,mocap_rigid_body]
        |                                          |
        v                                          v
cuvslam_localization_adapter              mocap_localization_adapter
  T[odom,base_link]                         T[mocap_world,base_link]
        |                                          |
        +------------------+-----------------------+
                           |
                    comparison/logger
             (一次对齐并在 localization epoch 内锁定)
                           |
                 [future source selector]
              launch-time: cuvslam | mocap
                           |
                 /localization/odometry
                           |
           mavros_external_odometry_gateway
                （唯一允许的 PX4 writer）
                           |
                  MAVROS 2.14 -> PX4 EKF2
```

两个定位源是同级关系。禁止把动捕接在 cuVSLAM 后面，也禁止让两个 adapter 分别向 MAVROS 发布。源选择只允许在 launch 时完成；首版不支持飞行中热切换或故障自动接管。

核心 adapter 不得创建任何 `/mavros/*` publisher，不得调用 arming、mode、OFFBOARD 或 PX4 参数服务。

## 5. 坐标系合同

### 5.1 目标 frame 树

```text
map                         项目世界系，目标为 ENU
└── odom                    连续局部里程计，不允许跳变
    └── base_link           飞机质心/控制参考点，FLU
        ├── fcu_imu         飞控 IMU 运行 frame，目标 ROS 表达为 FLU
        └── camera_link     D435 机身坐标，FLU
            ├── camera_infra1_optical_frame
            ├── camera_infra2_optical_frame
            └── camera_depth_optical_frame
```

`MAV_FRAME_LOCAL_NED`、`MAV_FRAME_LOCAL_FRD` 和 `MAV_FRAME_BODY_FRD` 是 MAVLink `ODOMETRY` 消息中的 frame enum，不是天然存在的 ROS TF frame。核心 adapter 不发布或维护同名 TF。

### 5.2 frame 定义

| Frame | 轴向 | 原点/用途 | 当前状态 |
| --- | --- | --- | --- |
| `map` | ENU：x 东、y 北、z 上 | 任务目标和 PX4 ROS 侧对齐世界 | 目标合同；`T[map,odom]` 来源待定 |
| `odom` | 右手、z-up；初始 yaw 可任意 | cuVSLAM 连续局部世界 | 已有连续输出；不得仅重命名为 `map` |
| `base_link` | FLU：x 前、y 左、z 上 | 飞机质心/控制参考点 | 目标合同；物理原点与外参待固定 |
| `fcu_imu` | 目标为 ROS FLU | 飞控 IMU 运行 frame/物理中心 | MAVROS 源码与静态重力样本支持 FLU；单轴实测和到质心关系待完成 |
| `camera_link` | FLU | D435 机身参考点 | 当前 cuVSLAM child frame |
| `mocap_world` | 固定、右手、z-up；水平 x/y 为实验室参考轴 | 动捕本地世界，不代表地理东/北 | 三轴实测候选；与 raw `world` 暂按 identity 对齐 |
| `mocap_rigid_body` | FLU：x 前、y 左、z 上 | `droneyee207` 刚体虚拟原点 | 轴配置已确认；原点与 `base_link` 重合为未测量安装假设 |
| optical frames | x 右、y 下、z 前 | 图像投影坐标 | ROS 相机标准 |
| `mavros_local` | 右手、z-up；水平轴由显式对齐合同定义 | gateway 内部候选 parent | 目标合同；不是 raw `odom` 的别名，也不直接发送给 MAVROS |
| `MAV_FRAME_LOCAL_NED` | x 北、y 东、z 下 | 地理/导航对齐的 MAVLink local parent enum | 仅 MAVROS/PX4 消息边界 |
| `MAV_FRAME_LOCAL_FRD` | x 为初始化/对齐定义的前、y 右、z 下；建立后相对世界固定 | 非地理对齐的 MAVLink local parent enum | 仅 MAVROS/PX4 消息边界；不得写成“北/东” |
| `MAV_FRAME_BODY_FRD` | x 当前机头前、y 右、z 下；随机体旋转 | MAVLink child enum | 仅 MAVROS/PX4 消息边界 |

### 5.3 轴映射参考

这些映射只用于审计和测试，不应以散落的符号取反实现：

```text
ROS optical -> FLU body convention:
  (x_flu, y_flu, z_flu) = (z_optical, -x_optical, -y_optical)

ROS ENU -> PX4 NED:
  (x_ned, y_ned, z_ned) = (y_enu, x_enu, -z_enu)

ROS FLU -> PX4 FRD:
  (x_frd, y_frd, z_frd) = (x_flu, -y_flu, -z_flu)
```

实际相机到机体转换必须使用批准的完整外参，而不是只使用上述轴排列。具体 MAVROS plugin 分支采用 `LOCAL_NED` 还是 `LOCAL_FRD`，以及它如何解释 ROS parent frame，必须按固定源码 commit 和台架结果记录；上面的 ENU/NED 公式不能代替这项审计。

### 5.4 数值与单位约定

- 长度使用 m，时间使用 s/ns，线速度使用 m/s，角速度使用 rad/s，角度计算使用 rad；
- 四元数按 ROS 字段顺序 `(x, y, z, w)` 存储，使用前必须归一化；`q` 与 `-q` 视为同一旋转；
- 旋转矩阵和 `T[A,B]` 均采用列向量左乘语义 `p[A] = T[A,B] * p[B]`；
- pose/twist covariance 使用 ROS 6x6 row-major 排列 `(x, y, z, rot_x, rot_y, rot_z)`；具体扰动左乘/右乘、参考点和表达 frame 仍必须由来源合同确认；
- 禁止在消息边界使用 degree、mm、g 或未标单位的配置值。

## 6. 变换记号与数学合同

本仓库统一使用：

```text
T[A,B]：把 B frame 中的点坐标变换到 A frame
p[A] = T[A,B] * p[B]
```

当前 cuVSLAM pose 为 `T[odom,camera_link]`。目标机体 pose 为：

```text
T[camera_link,base_link] = inverse(T[base_link,camera_link])
T[odom,base_link] = T[odom,camera_link] * T[camera_link,base_link]
```

当前动捕 pose 固定解释为 `T[world,mocap_rigid_body]`。影子机体 pose 为：

```text
T[mocap_rigid_body,base_link] = inverse(T[base_link,mocap_rigid_body])
T[mocap_world,base_link] =
  T[mocap_world,world]
  * T[world,mocap_rigid_body]
  * T[mocap_rigid_body,base_link]
```

首版两个静态变换均采用 identity，但批准状态不同：`T[mocap_world,world]=I` 是本地实验室 frame 的显式命名；`T[base_link,mocap_rigid_body]=I` 只是安装假设。两者都不得被外推成“动捕已经与地理 ENU 或 PX4 local frame 对齐”。

当且仅当存在经过批准且健康的 `T[map,odom]`（TF parent=`map`、child=`odom`）时，才能生成：

```text
T[map,base_link] = T[map,odom] * T[odom,base_link]
```

MAVROS 候选 parent 与 raw `odom` 不得共用名称。gateway 必须使用独立、在单次 localization epoch 内保持常量的对齐：

```text
T[mavros_local,base_link] = T[mavros_local,odom] * T[odom,base_link]
```

只有实验证明 `T[mavros_local,odom] = I` 时才允许省略该乘法；当前没有这项证据。

禁止执行以下伪转换：

```text
output.pose = input.pose
output.child_frame_id = "base_link"
```

相机与质心之间存在非零杠杆臂。旋转运动会改变两个参考点的线速度，因此不能只旋转三个线速度分量。必须先通过源码、解析定义和单轴实验识别输入速度对应的参考时刻、参考点和表达 frame，再选择正确的 rigid-body motion transform；不得在定义未确认时直接套用通用 adjoint。

协方差也必须先识别来源、误差状态、排列和表达 frame，再选择匹配的 Jacobian。只有这些定义确认后，才允许使用：

```text
Sigma_out = J * Sigma_in * J^T
```

当前 tracking odometry 中的 pose covariance 来自近期 pose 缓存的样本离散度，twist covariance 来自派生速度缓存的样本离散度；它们不是已经证明的 cuVSLAM 估计器误差 covariance。SDK 的 pose estimate covariance 位于另一个 pose-covariance topic，若未来使用，必须先完成时间同步和误差定义审计。

默认 covariance 策略为 `reject`：在来源、排列、扰动方向、表达 frame 和统计质量获批准前，adapter 的 odometry `messages=0`，但 diagnostics 继续发布。这里的“零”绝不表示发布一条数值全零的 odometry。禁止把当前窗口离散度传播后交给通用消费者或 PX4 EKF2。

## 7. 输入合同

### 7.1 cuVSLAM odometry

| 项目 | 当前期望 |
| --- | --- |
| Topic | `/visual_slam/tracking/odometry` |
| Type | `nav_msgs/msg/Odometry` |
| `header.frame_id` | `odom` |
| `child_frame_id` | `camera_link` |
| Timestamp | 非零、严格递增、与相机/IMU 同一 ROS 时间域 |
| Pose | `T[odom,camera_link]` |
| Twist | 时间窗口最早 pose 到最新 pose 的局部差分；参考时刻和标准 twist 等价性待单轴实测确认 |
| Covariance | tracking odometry 提供窗口样本离散度；默认拒绝作为估计器 covariance |

输入 frame 不匹配时必须拒绝消息，禁止通过参数把错误 frame 名称“配置正确”。

### 7.2 tracking 与 diagnostics

核心 adapter 同时消费：

- `/visual_slam/status`；
- cuVSLAM 运行 diagnostics；
- aligned FCU IMU diagnostics；
- adapter 所需 TF 的可用性和唯一父节点状态。

允许发布的 tracking 状态必须绑定具体的 `isaac_ros_visual_slam_interfaces` 版本。当前实测 `vo_state=1` 表示正常跟踪，但实现前仍需核对消息定义，不能在代码中留下无来源的裸数字。

status、diagnostics 与 odometry 不是原子同步消息。配置必须定义各健康状态的最大允许年龄。

### 7.3 静态外参

当前已知：

- 当前运行 TF 中存在 parent=`camera_infra1_optical_frame`、child=`fcu_imu` 的标定关系；按本文记号对应 `T[camera_infra1_optical_frame,fcu_imu]`。导入实现前必须同时记录 Kalibr 原始字段名、原矩阵方向和取逆过程，不能依赖箭头文字猜测；
- RealSense 驱动提供 `camera_link` 与 optical frames 的静态 TF；导出配置时同样必须归一化成明确的 `T[A,B]`；
- `/mavros/imu/data_raw` 的 frame 名称是 `base_link`，但这个名称本身不能证明其原点位于飞机质心；
- `T[base_link,fcu_imu]` 尚未测量或批准。

因此，在 `T[base_link,fcu_imu]` 完成前，生产模式不得宣称已经得到 `T[odom,base_link]`。

在所有来源方向均按本文记号固化后，目标链式关系为：

```text
T[base_link,camera_link] =
  T[base_link,fcu_imu]
  * inverse(T[camera_infra1_optical_frame,fcu_imu])
  * inverse(T[camera_link,camera_infra1_optical_frame])
```

这条公式只固定乘法方向，不批准任何当前数值；每一项仍需 provenance、单位、设备序列号和独立复核。

### 7.4 全局对齐 `T[map,odom]`

当前 odometry-only cuVSLAM 明确不发布 parent=`map`、child=`odom` 的 TF。本仓库必须指定 `T[map,odom]` 的唯一来源，例如：

- 经过验证的启动时静态对齐；
- 独立全局定位器；
- 未来受控的全局校正模块。

没有对齐来源时：

- 只有 `T[odom,base_link]`、标准 twist 和 covariance 策略均获批准时，`/localization/odometry` 才可在 `odom` 中发布；
- `map` pose 和 YOPO 世界目标适配必须保持关闭；
- 禁止把 `frame_id` 字符串直接改成 `map`；
- identity 只能用于明确标记的台架测试，不能作为默认生产假设。

### 7.5 对齐变换所有权

`T[map,odom]` 与 `T[mavros_local,odom]` 都必须各自只有一个 authority。该 authority 可以是带 provenance 的静态配置、独立定位器或未来的对齐节点，但必须明确记录：

- parent/child、矩阵方向、来源和配置 hash；
- localization epoch 与生效时间；
- 是否允许运行中改变，以及改变时的 reset 协议；
- TF 或消息的唯一 publisher；
- stale、冲突和重启时的处理方式。

核心 adapter、YOPO bridge 和 MAVROS gateway 都只能消费已批准的对齐，不能各自估算或缓存一套隐式 yaw offset。

## 8. 输出合同

### 8.1 `/localization/odometry`：唯一标准状态

| 字段 | 合同 |
| --- | --- |
| Type | `nav_msgs/msg/Odometry` |
| `header.stamp` | 完整继承输入采样时间戳 |
| `header.frame_id` | `odom` |
| `child_frame_id` | `base_link` |
| Pose | `T[odom,base_link]`，表达于 `odom` |
| Twist | 获批准后：`base_link` 原点的速度，表达于 `base_link` |
| Pose covariance | 获批准后：按已固定的误差状态/扰动约定传播，并表达于 `odom` |
| Twist covariance | 获批准后：表达于 `base_link`，按匹配 Jacobian 传播 |

这个 topic 不得为 YOPO、SO3 或 MAVROS 分别改变语义。消费者需要其他表达时，必须使用独立桥。缺少批准的 `T[base_link,camera_link]`、标准 twist 定义或 covariance 策略时，允许实现接口和影子计算，但运行态必须发布零条规范 odometry。

### 8.2 `/state/odom`：YOPO 临时兼容接口

当前 YOPO ROS 2 代码把 `twist.twist.linear` 当作世界系速度，并且只校验可配置的 `header.frame_id`，不校验 `child_frame_id`。`plan_from_reference=true` 已配置，但该参数只改变重规划起点，不改变坐标语义。

兼容期间的完整线上合同如下：

| 字段 | 临时合同 |
| --- | --- |
| Type | `nav_msgs/msg/Odometry` |
| `header.stamp` | 继承 `/localization/odometry` 的采样时间 |
| `header.frame_id` | `map` |
| `child_frame_id` | `base_link` |
| Pose | `T[map,base_link] = T[map,odom] * T[odom,base_link]` |
| `twist.linear` | `base_link` 原点的线速度，旋转后表达于 `map` |
| `twist.angular` | 机体角速度，旋转后表达于 `map`；YOPO 当前不消费，但不得留下另一套 frame 语义 |
| Pose covariance | 仅在获批准后，按 `map` pose 合同传播 |
| Twist covariance | 仅在获批准后，整块 6x6 与 world-expressed twist 一致地传播 |

这里 `child_frame_id=base_link`，但 twist 故意表达于 `map`，违反标准 `nav_msgs/Odometry` 的通常约定；这正是它只能作为 legacy 接口的原因。因此：

- `/state/odom` 只能由 `yopo_legacy_bridge` 发布；
- 必须在 diagnostics 中标记 `legacy_world_velocity=true`；
- 不能被 MAVROS、控制器或其他通用 odometry 消费者复用；
- 在 `T[map,odom]` 无效时停止发布；
- 在上游 stale、时间回退、TF/变换失败或 covariance 策略失效时同样停止发布，不得定时重发最后状态；
- 长期方案是修改 YOPO 正确消费 child-frame twist，然后删除该兼容桥。

标准 `/localization/odometry` 不能通过简单 remap 直接变成 `/state/odom`。

### 8.3 `/localization/mavros_candidate`：飞控候选影子输出

`mavros_external_odometry_gateway` 首先只生成影子消息：

| 字段 | 合同 |
| --- | --- |
| Type | `nav_msgs/msg/Odometry` |
| `header.frame_id` | `mavros_local`，gateway 内部 frame，不是 MAVROS 输入 frame |
| `child_frame_id` | `base_link`，FLU |
| Pose | `T[mavros_local,base_link] = T[mavros_local,odom] * T[odom,base_link]` |
| Twist | 表达于 `base_link` |
| Pose covariance | 获批准后，传播并表达于 `mavros_local` pose 合同 |
| Twist covariance | 获批准后，表达于 `base_link`；未批准时 candidate 也不得发布 |
| Timestamp | 继承规范状态采样时间 |

影子模式用于检查坐标、频率、时间、协方差和故障门禁，不向 PX4 发送数据。`T[mavros_local,odom]` 在单次 localization epoch 内必须保持常量；候选禁止使用可能因全局校正而跳变的 `T[map,odom]`。

当前原始 cuVSLAM `odom` 的初始 yaw 可以任意。它不能因为名字叫 `odom` 就直接视为 MAVROS 所需的 parent frame。gateway 启用前必须固定并验证 `T[mavros_local,odom]`；这项连续局部对齐与全局 `T[map,odom]` 是两个不同合同。影子 candidate 不能通过 remap 直接发送到 `/mavros/odometry/out`。

### 8.4 `/mavros/odometry/out`：默认禁止

输出拓扑按模式固定，不允许“节点存在但是否发送不确定”：

| 模式 | gateway | `/localization/mavros_candidate` | `/mavros/odometry/out` |
| --- | --- | --- | --- |
| default | 不启动 | publisher=0，messages=0 | publisher=0，messages=0 |
| shadow | 启动 | publisher=1 | publisher=0，messages=0 |
| bench transmit | 显式启用且门禁通过 | publisher=1 | publisher=1；仅健康输入时发送 |

只有单独 gateway、显式 bench 配置和独立验收同时满足时，才允许进入 bench transmit：

- MAVROS 输入消息的 `header.frame_id` 必须是固定 plugin commit 明确支持的值；不得把内部 `mavros_local` 字符串直接发给 MAVROS；
- parent 必须由连续且已验证的 `T[mavros_local,odom]` 生成，child 必须表示 ROS FLU `base_link`；
- `/mavros/odometry/out` 只能存在一个 publisher；
- 禁止手工对四元数 `y/z` 取反；
- 禁止手工提前转换为 NED/FRD；
- 当前 MAVROS 2.14.0 工作副本观察到 ROS->FCU 订阅端是 `~/out`，相关分支使用 `MAV_FRAME_LOCAL_FRD`/`MAV_FRAME_BODY_FRD` enum；阶段 4 必须补齐精确 commit、文件、函数、frame-id 分支和转换函数记录；
- MAVLink frame enum 不是 ROS TF frame。源码审计也不代替台架测试；只有插件加载、topic 方向、消息 frame、变换结果和实际 MAVLink 输出通过验收后，才能把该发送路径视为运行合同；
- gateway 不得调用 arming、mode、OFFBOARD 或 PX4 参数服务。

发布成功或 PX4 收到消息不等于 EKF2 已正确融合。融合状态必须通过 PX4 estimator status、innovation 和 ULog 独立验证。

## 9. 时间合同

每条输出必须保留输入 `header.stamp`。以下情况必须拒绝并计数：

- 零时间戳；
- 重复或非严格递增时间戳；
- 时间戳明显位于未来；
- source stamp 已过期；
- receipt time 已过期；
- 输入跨时钟域；
- 定位节点重启后时间或轨迹无声明地重置。

禁止使用 `now()` 替换采样时间，也禁止在上游停止后周期性重发最后一帧。

所有时间阈值在完成 Jetson 延迟测量前都是可配置候选值，不能在 README 中声称已有生产上界。

## 10. Fail-Closed 状态机

任一条件成立时，核心 adapter、legacy bridge 或 gateway 必须停止发布受影响的状态输出：

- tracking 状态无效、缺失或过期；
- odometry 缺失、过期、时间倒退或跨时钟域；
- `header.frame_id` 或 `child_frame_id` 与合同不符；
- 必需 TF 不存在、存在重复父节点或来源未经批准；
- position、orientation、twist 或 covariance 含 NaN/Inf；
- quaternion 近零、无法归一化或范数超出允许范围；
- covariance 非有限、非对称、非半正定或违反配置策略；
- 检测到未声明的大跳变、定位重置或对齐变换改变；
- 内部变换、时间或配置校验失败。

故障影响范围必须明确，不能让一个未使用的全局对齐阻断健康的局部 odometry：

| 故障 | 必须停止的输出 | 不应被无条件停止的输出 |
| --- | --- | --- |
| cuVSLAM tracking/odometry、时间、输入 frame、`T[base_link,camera_link]` 或规范 covariance 失效 | `/localization/odometry`、`/state/odom`、candidate、MAVROS output | diagnostics |
| `T[map,odom]` 缺失、改变或过期 | `/state/odom` | 健康的 `/localization/odometry`；不依赖它的 local candidate |
| `T[mavros_local,odom]` 缺失、改变或过期 | candidate、MAVROS output | `/localization/odometry`；仅依赖 `T[map,odom]` 的 YOPO bridge |
| candidate 内部变换或 candidate 配置不匹配 | candidate、MAVROS output | adapter 与 YOPO 输出 |
| MAVROS plugin 合同不匹配或存在重复 writer | MAVROS output | candidate、adapter 与 YOPO 输出 |

进入故障状态后：

1. 立即停止发布，不重发旧状态；
2. diagnostics 进入 ERROR/STALE，并给出机器可解析的原因和累计计数；
3. diagnostics 始终继续发布；不得因状态门禁而一起关闭；
4. 暂时性 stale/tracking loss、短时 TF 缺失以及单帧非有限值/坏四元数/坏 covariance 可配置为自动恢复，但必须先丢弃坏消息，并连续 `recovery_consecutive_samples` 帧重新满足合同；具体数值必须由 Jetson 数据确定；
5. frame 名称不匹配、内部变换/静态配置错误在配置修正前不得恢复；
6. 时间回退、定位 epoch/reset、对齐变换运行中改变、重复 MAVROS writer、配置/合同 hash 改变属于锁存故障，要求人工复位或进程重启；
7. 恢复不得静默改变 `T[map,odom]`、`T[mavros_local,odom]` 或定位原点。

## 11. 已确认事实与未决项

### 11.1 已由源码或配置确认

- YOPO 模型和旧 SO3 控制内部使用右手 z-up、x 前、y 左、z 上的机体约定；
- 当前 YOPO ROS 2 配置已设置 `plan_from_reference=true`；
- 当前 YOPO 将线速度按世界系使用；
- cuVSLAM wrapper 将 `header.frame_id` 设为配置的 odom frame，将 `child_frame_id` 设为配置的 `base_frame`；当前运行值分别为 `odom` 与 `camera_link`；
- 当前项目配置的 cuVSLAM `base_frame=camera_link`；
- cuVSLAM 使用窗口首末局部 pose 差计算速度；这尚未证明等价于当前时刻的标准 child-frame twist；
- tracking odometry covariance 是缓存 pose/派生速度的窗口样本离散度，不是已批准的估计器误差 covariance；
- 当前 odometry-only 配置关闭 mapping、回环以及 parent=`map`、child=`odom` 的 TF 发布。

### 11.2 MAVROS 工作副本观察（证据待固化）

- 当前 MAVROS 2.14 工作副本的 `odometry` plugin 订阅 `~/out`，并在 ROS->FCU 路径处理 parent/child 表达；
- 该观察只适用于这条 plugin 路径，不外推到所有 MAVROS 接口；
- 在记录精确 commit、文件、函数和 frame-id 分支之前，它不能作为可复现的运行合同。

### 11.3 已由当前真机日志观察

- 已观察到约 90 Hz odometry、约 170 Hz aligned FCU IMU、严格递增时间戳和有效受控运动响应；
- mapping-on 联合链路曾连续运行约 17 分 48 秒；最新 odometry-only 版本仍需 Jetson A/B。

这些观察证明当前链路能产生连续数据，不等价于坐标、协方差、PX4 融合或生产稳定性已验收。

### 11.4 仍是阻塞项

- `base_link` 质心/控制参考点的物理定义；
- `T[base_link,fcu_imu]` 的真实平移和旋转；
- 完整 `T[base_link,camera_link]` 的推导与独立复核；
- cuVSLAM twist 轴向和杠杆臂公式的单轴实测；
- cuVSLAM 窗口速度的参考时刻以及到标准瞬时 twist 的处理策略；
- cuVSLAM covariance 的误差定义和统计质量；
- `T[map,odom]` 的唯一来源、初始化和重置策略；
- `T[mavros_local,odom]` 的定义、初始化和不变性策略；
- YOPO 从 legacy world velocity 迁移到标准 odometry；
- MAVROS 2.14 checkout 的精确 commit、plugin frame 分支和转换函数审计记录；
- PX4 external vision 延迟、协方差、创新和复位验收；
- gateway 禁用、影子、非融合传输和融合阶段的真机测试。

## 12. 推荐仓库结构

```text
localization_state_adapter/
├── README.md
├── docs/
│   ├── coordinate_contract.md
│   ├── topic_contract.md
│   ├── calibration_provenance.md
│   ├── safety_case.md
│   └── acceptance.md
├── localization_contracts/
│   ├── include/ or localization_contracts/
│   └── test/
├── localization_adapter_interfaces/
│   └── msg/ShadowPoseCandidate.msg
├── cuvslam_localization_adapter/
│   ├── src/ or cuvslam_localization_adapter/
│   ├── config/
│   ├── launch/
│   └── test/
├── mocap_localization_adapter/
│   ├── include/ and src/
│   ├── config/
│   ├── launch/
│   └── test/
├── yopo_legacy_bridge/
│   ├── src/ or yopo_legacy_bridge/
│   ├── config/
│   └── test/
├── mavros_external_odometry_gateway/
│   ├── src/ or mavros_external_odometry_gateway/
│   ├── config/
│   ├── launch/
│   └── test/
├── localization_bringup/
│   └── launch/
└── tools/
    ├── bag_contract_probe
    └── fault_injector
```

建议将 adapter 与 gateway 放在同一仓库以共享合同和数学测试，但保持不同 ROS package、不同进程和不同 launch。

## 13. 实施阶段

### 阶段 0：冻结合同

- 固定 frame、原点、轴向和变换记号；
- 测量 `T[base_link,fcu_imu]`；
- 推导并复核 `T[base_link,camera_link]`；
- 确认 cuVSLAM twist/covariance、`T[map,odom]` 和 `T[mavros_local,odom]` 来源；
- 将所有未决项留为显式 blocker，不使用默认猜测。

### 阶段 1：纯数学与单元测试

- identity、纯平移、单轴 `+/-90 deg` 和组合 pose 变换；
- 非零杠杆臂与角速度下的 twist 变换；
- 在误差状态获确认后，测试对应的 6x6 covariance Jacobian 传播；
- quaternion `q/-q` 等价、归一化和异常输入；
- frame、时间戳、stale、重置和恢复状态机；
- 随机 SE(3) 性质测试和正反变换闭环。

### 阶段 2：被动 adapter

- 只发布 `/localization/odometry` 和 diagnostics；
- 完成 rosbag 回放、故障注入和 Jetson 实时测试；
- 验证前/左/上平移、三轴旋转、静止漂移和已知距离；
- 连续运行至少 30 分钟；
- 不启动 YOPO 控制、不发布 MAVROS external vision。

### 阶段 3：YOPO PASSIVE

- 启用 `yopo_legacy_bridge`；
- 保持 YOPO `output_enabled=false`；
- 分别给出 `+X/+Y/+Z` 目标并检查轨迹方向；
- 验证世界速度、姿态、深度图方向和时间新鲜度；
- 完成后优先修改 YOPO，消除 legacy world velocity。

### 阶段 4：MAVROS gateway 影子模式

- 只发布 `/localization/mavros_candidate`；
- 在持续健康输入下证明 `/mavros/odometry/out` publisher count 与消息数均严格为零；
- 检查 `T[mavros_local,odom]`、时间、频率、frame、协方差和故障锁存；
- 固定 MAVROS commit，并记录 `header.frame_id` 到 MAVLink frame enum 的实际源码分支。

### 阶段 5：PX4 非融合传输

- 拆桨、未解锁、固定平台；
- 使用独立的非默认 bench 配置显式进入 `bench transmit`，并确认 `/mavros/odometry/out` publisher=1；
- 只有全部健康门禁通过时允许产生消息，任一故障立即停止消息；验收后恢复 `shadow` 或 `default` 模式；
- 验证 MAVROS plugin、topic 方向、TF 和 MAVLink 收包；
- PX4 必须保持 external vision 融合关闭；
- 完成独立日志、验收记录和签字门。

### 阶段 6：PX4 EKF2 单项融合（仓库外系统集成验收）

- 本阶段不向本仓库增加 EKF2 参数管理或传感器选择代码；本仓库只引用外部验收记录；
- 拆桨、未解锁、固定平台；
- 按独立安全流程一次只启用一个 EKF2 external vision 融合项；
- 检查 estimator status、innovation、延迟、超时、复位和恢复；
- 飞行测试属于后续独立项目阶段，不是本仓库首版验收。

## 14. 测试门禁

### 14.1 每个 PR 的合并门禁

每个修改只需满足其影响范围内的可执行门禁，不能要求阶段 0/1 的提交提前完成后续真机阶段：

- 单元测试覆盖所有基轴和符号；
- 随机 SE(3) pose/twist 性质测试；启用或修改 covariance 路径时，增加对应 covariance 性质测试；
- 输入 frame、NaN/Inf、坏四元数以及当前启用策略下的坏 covariance 拒绝测试；
- 时间戳零值、重复、回退、未来、stale 和恢复测试；
- tracking lost、TF 缺失、定位重置和大跳变故障注入；
- rosbag 回放结果可重复；
- 默认 launch 不创建 `/mavros/odometry/out` publisher 的测试；
- 文档、配置、消息合同与测试同步更新。

### 14.2 阶段退出验收

| 阶段 | 额外退出条件 |
| --- | --- |
| 2 | Jetson 单轴刚体运动、静止漂移、已知距离、故障注入和至少 30 分钟被动运行 |
| 3 | YOPO PASSIVE 的 `+X/+Y/+Z` 目标、世界速度、深度方向和无控制输出验证 |
| 4 | shadow candidate 审计、`/mavros/odometry/out` publisher=0/messages=0、gateway 锁存测试 |
| 5 | 拆桨未解锁条件下的 plugin、topic、frame enum、转换结果和 MAVLink 收包验证；EKF2 不融合 |
| 6（仓库外） | 单项 EKF2 融合、innovation、延迟、超时、reset 和 ULog 验收；结果作为外部证据链接回本仓库 |

“topic 存在”“频率正常”或“PX4 收到消息”都不能单独作为坐标或融合正确的验收证据。

## 15. 目标平台与兼容基线

当前审计基线：

| 组件 | 基线 |
| --- | --- |
| ROS | ROS 2 Humble |
| Isaac ROS Visual SLAM | `v3.2-15` / NVIDIA wrapper `e31f4cc` |
| 项目 cuVSLAM 集成 | `u5-4/fcu-imu-cuvslam-integration@04d7b9c` |
| MAVROS | `2.14.0`；精确 checkout commit 与 `odometry` plugin frame 分支待阶段 4 固定 |
| PX4 | `1.15.4` 实测平台 |
| 相机 | Intel RealSense D435i，factory-rectified stereo |
| 动捕客户端 | `u5-4/vrpn_client_ros2@1b9731c`，server `192.168.151.168:3883` |
| 动捕刚体 | `droneyee207`，`/droneyee207/pose`，现场约 120 Hz |

本仓库只通过标准 ROS 2 消息和 TF 与 Isaac ROS Visual SLAM 通信，不构建、不包含、不链接 cuVSLAM core 二进制或头文件。

## 16. 开发和变更规则

- 坐标变换必须集中在合同库和 adapter/gateway 边界，禁止在 YOPO、SO3 或 launch 中再次散落符号取反；
- 每一个新增 frame 都必须记录父 frame、原点、轴向、单位、来源、发布节点和验证方法；
- 每一份外参必须记录设备、标定文件、变换方向、单位、来源 commit 和批准状态；
- 任何修改 `frame_id`、twist、covariance、timestamp 或 gateway enable 逻辑的提交都必须附带测试；
- 默认配置和默认 launch 必须保持被动，不得向 PX4 发布；
- 不允许通过修改 MAVROS 源码绕过本仓库合同；
- 不允许把 candidate、bench 或 identity alignment 配置标记为 production；
- 未通过真机验收的功能必须在 README 和 release notes 中保持“未验证”状态。

## 17. 当前发布边界

阶段 0 和阶段 1 只允许定义接口、实现影子计算和运行测试。缺少批准外参、twist 或 covariance 策略时，规范 odometry 必须为 `messages=0`，但 diagnostics 必须继续发布；禁止发送数值全零的占位 odometry。动捕分支允许发布下面这个源私有、类型受限的影子 pose，用于录包和对比：

```text
/localization/shadow/mocap/assumed_base_pose
  type: localization_adapter_interfaces/msg/ShadowPoseCandidate
  parent: mocap_world
  semantic child: base_link
  world_alignment_approved: false
  extrinsic_approved: false
  source_configuration_validated: false
  capture_time_validated: false
```

这个话题不是 `/localization/odometry`，没有 twist/covariance，并且消息类型与 MAVROS pose/odometry 输入不兼容。每条消息都显式携带合同 ID、source frame、安装假设和时间语义，不能通过简单 remap 冒充飞控输入。

只有阶段 2 被动验收通过后，默认定位启动路径才允许提供：

```text
/localization/odometry
/diagnostics
```

以下接口当前都属于未来阶段，不得出现在默认启动路径：

```text
/state/odom
/localization/mavros_candidate
/mavros/odometry/out
```

README 暂不提供生产启动命令。只有代码、配置、单元测试、rosbag 回放和 Jetson 被动验收全部完成后，才能加入正式 build、launch 和停止流程。

## 18. 动捕双源扩展状态

### 18.1 已获得的现场证据

2026-07-22 在 Jetson 容器内观测到：

- `/droneyee207/pose` 类型为 `geometry_msgs/msg/PoseStamped`，唯一 publisher 为 `/vrpn_client_node`；
- offered QoS 为 reliable、volatile；Humble/Fast DDS 的 endpoint introspection 将 history/depth 报为 `UNKNOWN`，平均频率约 120 Hz；
- `header.frame_id=world`；
- 向上移动时 raw z 增加；从上往下看逆时针旋转时四元数表现为正 z 旋转；
- 受控水平移动得到向前增量 `(+0.787,+0.109,+0.087) m`，向左增量 `(-0.110,+0.770,+0.026) m`；两个水平投影夹角约 `90.2 deg`；
- 刚体局部轴在动捕软件中配置为 x 前、y 左、z 上。

这些结果支持“固定实验室世界为右手 z-up、刚体为 FLU”的候选合同。测试期间姿态仍有约 `8.5 deg` 和 `17.7 deg` 的变化，因此它不构成精密外参标定，也不能证明实验室 x/y 是地理东/北。

当前 VRPN client mainloop 配置为 60 Hz，而现场 topic throughput 约 120 msg/s；源码允许一次 mainloop 排出多条 tracker report，因此实际时间序列可能呈现“批内接近 0 ms、批间约 16.7 ms”。120 只能描述 callback 吞吐量，不能解释为均匀的 120 Hz 光学采样，更不能用当前 callback stamps 直接微分速度。adapter diagnostics 会分别报告最小/最大 stamp gap 和亚毫秒 gap 比例。

### 18.2 VRPN 数据边界

对 `vrpn_client_ros2@1b9731c` 的源码审计表明：

- position 和 quaternion 原样复制，没有轴交换、符号取反、外参或 `world -> map` 转换；
- 当前 `use_server_time=false`，header stamp 是 Jetson 上 VRPN callback 执行时的 ROS system time，不是光学曝光时刻，包含网络和调度延迟；
- pose、twist 和 accel 是独立 callback，不能按同一 stamp 拼成一个状态；
- 当前 `twist.angular` 和 `accel.angular` 忽略 VRPN 的对应时间间隔，不具备 ROS `rad/s` 或 `rad/s^2` 语义；
- 上游没有 covariance，TF 发布路径未启用，同一 sender 的 sensor id 没有保留。

因此首版只消费 PoseStamped 的 stamp、frame、position 和 quaternion。所有 VRPN twist/accel 字段都禁止进入 YOPO、规范定位或 MAVROS。

当前 adapter 能绑定运行时 publisher 的 FQN、GID、消息类型，以及 endpoint graph 能可靠提供的 QoS 证据。Reliability 必须为 reliable，durability 必须为 volatile；history 只接受明确的 keep-last，或 Humble/Fast DDS introspection 的 `UNKNOWN`。`UNKNOWN` 仅表示 RMW 未报告，不能证明上游确实使用 KeepLast，diagnostics 必须将该回退显式报告。该回退只允许用于当前 shadow 阶段；`mocap_primary` 或任何 PX4 输出阶段必须改用受 GID 绑定的上游 manifest/固定配置证据，不得继续把 `UNKNOWN` 当作充分证据。VRPN 进程尚未发布自身 git revision、配置 hash 或 `use_server_time` 诊断证据，因此消息中的 revision/time 字段使用 `expected_*` 命名，并固定携带 `source_configuration_validated=false`；这项状态在增加上游 manifest 前不得升级。

合同 ID `droneyee207_mocap_shadow_20260722_v2` 专门标识上述 shadow-only RMW history 回退语义；旧的无后缀 ID 对应严格要求 graph 报告 keep-last 的已废弃行为，两者不得混用于同一份录包或验收证据。

### 18.3 目标运行模式

| 模式 | PX4 主定位源 | cuVSLAM | 动捕 | 自动接管 |
| --- | --- | --- | --- | --- |
| `cuvslam_primary` | cuVSLAM | 主源 | 不要求启动 | 禁止 |
| `mocap_primary` | 动捕 | 持续运行，仅统一坐标后对比和录包 | 主源 | 禁止 |

两种模式最终都必须经过同一个 source selector 和同一个 MAVROS gateway。主源失效时立即停止 external odometry：`mocap_primary` 中 cuVSLAM 故障只能降低对比能力，不能切换成飞控定位源；动捕故障则必须停止该模式的飞控定位输出。

为诚实观察 cuVSLAM 漂移，`T[mocap_world,odom]` 只允许在每次 localization epoch 开始时估计一次并锁定。禁止持续重新对齐，因为那会把待测漂移消掉。任一来源重启、GID 改变或定位重置都结束当前 epoch。

### 18.4 当前分支实现范围

分支 `feat/mocap-localization-shadow` 当前只实现：

- `mocap_localization_adapter` C++17 节点；
- 固定 tracker、publisher、GID、frame、时间和数值门禁；
- identity 安装假设下的 source-private `ShadowPoseCandidate` 影子候选；
- diagnostics、数学测试和 ROS graph 禁止边界测试；
- 不创建 `/localization/odometry`、`/state/odom`、TF 或任何 `/mavros/*` publisher。

本分支尚未实现 source selector、cuVSLAM/动捕一次性对齐、误差统计或 MAVROS gateway。Jetson 编译和被动运行验证通过前，不得把本节状态改成可飞行。

开发验证命令：

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --symlink-install \
  --packages-up-to mocap_localization_adapter \
  --event-handlers console_direct+

source install/setup.bash
colcon test \
  --packages-select \
    localization_contracts \
    localization_adapter_interfaces \
    mocap_localization_adapter \
  --event-handlers console_direct+
colcon test-result --verbose
```

影子节点只能在 VRPN 客户端已启动后单独运行：

```bash
ros2 launch mocap_localization_adapter mocap_adapter_shadow.launch.py
```

运行后必须确认 `/localization/odometry`、`/state/odom`、`/tf`、`/tf_static` 和 `/mavros/*` 没有来自该节点的 publisher。

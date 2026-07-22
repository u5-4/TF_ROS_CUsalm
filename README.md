- YOPO 模型机体系确实是 `x前、y左、z上`，即 FLU；训练源码称其为 NWU：[yopo_dataset.py (line 89)](/C:/Users/10416/Documents/Codex/2026-07-16/w-m/work/YOPO-phase2/YOPO/policy/yopo_dataset.py:89)。
- SO3 内部也是 z-up，悬停补偿为 `+mg`：[SO3Control.cpp (line 70)](/C:/Users/10416/Documents/Codex/2026-07-16/w-m/work/YOPO-phase2/Controller/src/so3_control/src/SO3Control.cpp:70)。
- cuVSLAM ROS 输出已经转换成 `x前、y左、z上`：[cuvslam_ros_conversion.hpp (line 36)](/C:/Users/10416/Documents/Codex/2026-07-16/w-m/work/isaac_ros_visual_slam-v3.2-15/isaac_ros_visual_slam/include/isaac_ros_visual_slam/impl/cuvslam_ros_conversion.hpp:36)。
- 但是当前 cuVSLAM 输出的参考点是 `camera_link`，不是飞机质心。
- cuVSLAM 的 twist 是通过相邻 pose 的局部相对变换计算的，属于 child/base frame；YOPO 当前却把它当世界系速度使用：[test_yopo_ros.py (line 213)](/C:/Users/10416/Documents/Codex/2026-07-16/w-m/work/YOPO-phase2/YOPO/test_yopo_ros.py:213)。
- 作者旧控制器在发送 MAVROS 前手工将四元数 y、z 取反：[mavros_interface.h (line 157)](/C:/Users/10416/Documents/Codex/2026-07-16/w-m/work/YOPO-phase2/Controller/src/so3_control/include/so3_control/mavros_interface.h:157)。
- 标准 MAVROS 2.14 已经自动完成 ENU/FLU → NED/FRD。因此我们不能保留旧的 y、z 手工取反，否则会二次转换。
- `plan_from_reference=true` 当前 ROS2 配置已经设置。它只改变重规划起点，并不解决坐标转换问题。

**最终坐标合同**

| Frame          | 最终定义                                         |
| -------------- | ------------------------------------------------ |
| `map`          | ENU，x东、y北、z上；YOPO目标和 PX4 ROS侧对齐世界 |
| `odom`         | cuVSLAM连续局部世界，不跳变；初始 yaw 可能任意   |
| `base_link`    | FLU，x前、y左、z上；原点选飞机质心/控制参考点    |
| `fcu_imu`      | 飞控IMU中心，ROS表达为FLU                        |
| `camera_link`  | D435机身坐标，FLU                                |
| optical frames | x右、y下、z前                                    |
| `base_frd`     | PX4机体：`(x,-y,-z)`，只在MAVROS边界出现         |
| `map_ned`      | PX4世界：`(y,x,-z)`，只在MAVROS边界出现          |

需要由状态适配器计算：

```
T_odom_base = T_odom_camera * inverse(T_base_camera)
T_map_base  = T_map_odom * T_odom_base
```

最终接口分为：

```
/localization/odometry
  pose:  odom -> base_link
  twist: base_link
  标准 nav_msgs/Odometry

/state/odom
  pose: map -> base_link
  当前YOPO兼容阶段需要世界系线速度

/mavros/odometry/out
  pose: map -> base_link，ROS ENU/FLU
  twist: base_link
  交给标准MAVROS自动转换为NED/FRD
```


# 与 27UL_Sentry_ws 的对应关系

2026-11-15 查看了上游在线 `.gitmodules`、`ros2_services.yaml`、Makefile，以及本机 `27UL_Sentry_ws` 中的启动与设备配置。

| 参考工程 | 本工程 | 对应方式 |
|---|---|---|
| 根目录 `src/`、Makefile、scripts、ros2_services.yaml | 同名顶层结构 | 保留工作区管理方式 |
| `src/chassis_controllers` | 同名 C++ 包 | 实现四舵轮运动解算 |
| `src/universal_controller` | 同名 C++ 包 | 实现底盘、原有 Yaw/Pitch 与滑台 effort 控制 |
| `src/pb2025_robot_description` | `src/tunnel_sentinel_description` | 使用提供的 URDF 与本次简化几何 |
| `rmu_gazebo_simulator` 的 launch/world/config 组织 | `tunnel_sentinel_bringup` | 一车、一洞口的 Gazebo 场景 |
| EtherCAT/CAN 电机通信 | `gz_ros2_control` 关节接口 | 在电脑执行控制回路 |
| `/red_standard_robot1/cmd_vel` | 同名、同类型速度输入 | 便于后续连接导航程序 |

上游 `27UL_Sentry_chassis_controllers` 本次访问返回 HTTP 404。本工程的 C++ 实现为新写代码，读取的上游文件用于确定包职责、目录和启动组织方式。各本地包直接保存在当前工作区，后续可按团队需要分成独立仓库。

技术版本选择本机已安装的 ROS 2 Jazzy 与 Gazebo Harmonic。过去查看的比赛仿真使用 Humble/Fortress，其 world、模型与消息配置接入时需要逐项转换插件名称和启动参数。

当前交付包含四舵轮、原有 Yaw/Pitch、独立滑台、电控和简单通行。扩展模型将 Yaw 父连接移到滑台托架，原有机械参数由逐项检查确认一致。比赛全场、雷达/相机输出、Nav2、自瞄和裁判消息可以沿现有分包结构继续接入；对应 RMUL 2027 规则的场地尺寸与事件应依据正式资料填写。

参考来源：

- [27UL_Sentry_ws](https://github.com/unnc-aim/27UL_Sentry_ws)
- [上游子模块列表](https://github.com/unnc-aim/27UL_Sentry_ws/blob/main/.gitmodules)
- [上游服务配置](https://github.com/unnc-aim/27UL_Sentry_ws/blob/main/ros2_services.yaml)
- [Gazebo 与 ROS 配套版本](https://gazebosim.org/docs/harmonic/ros_installation/)
- [gz_ros2_control 接入说明](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)

本工程新写代码采用 Apache-2.0。`reference/tunnel_sentinel.urdf` 保留用户提供的内容；`config/gui.config` 来自本机 Gazebo 安装，调整了窗口与相机配置。

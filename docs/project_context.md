# 工程当前状态

更新日期：2026-11-15。

当前工作目录：`/home/mul/Tunnel_sentry`。旧目录 `/home/mul/Documents/ChatGPT/过洞哨兵` 作为兼容链接。仓库本地 Git 提交者为 VS Code 登录账号 `SHENJLYL`，邮箱 `248255012+SHENJLYL@users.noreply.github.com`。

用户确认的实车机构为“竖直滑台升降＋机械臂独立俯仰”。收缩高度按滑台最低、机械臂水平时的地面到整车最高点测量，目标为 25 cm 以内；水平展开高度目标为 40 cm。

现已加入独立滑台扩展：行程 160.75 mm，最低水平姿态几何高度 239.25 mm，最高水平姿态 400 mm。原有 Pitch 继续表示机械臂俯仰，Yaw 继续表示转向。机械臂抬头后的最高点随 Pitch 增加。

原始文件 `src/tunnel_sentinel_description/reference/tunnel_sentinel.urdf` 与 `/home/mul/Downloads/tunnel_sentinel.urdf` 一致。`scripts/generate_lift.py` 生成扩展 `urdf/tunnel_sentinel_lift.urdf`，将 `gimbal_yaw_joint` 的父 link 改接 `lift_carriage_link`；原有所有 link 定义、关节类型、偏移、轴向和惯性参数继续使用原值。零升降位置保持原有装配位置。

当前扩展有 17 个 link、16 个关节：原有 10 个旋转关节，新增 4 个固定关节和 2 个直线关节。其中托架按升降指令运动，中套筒按托架一半行程运动，二者的推力由同一 C++ 控制器计算。该同步驱动为套筒传动的测试近似，实车传动的力分配与损耗待机械参数补充。

近似几何资源包含原有 5 个 STL、3 个套筒 STL，以及 Odin1 和安装支架 STL，定义于 `meshes/primitives.yaml`。底盘中央开口 92×92 mm，三层套筒外宽 80／64／48 mm，完整伸出时轴向重叠 19.625 mm。滑台部件总质量按 1.30 kg 估计；Odin1 与支架另增 0.43 kg，扩展模型总质量 55.373409 kg。原有底盘惯性参数保留。

运行时将碰撞网格转换为同尺寸的盒体／圆柱，保留底盘开口和套筒内孔。包装 Xacro 添加 ros2_control、IMU 和 Gazebo 插件。原始 URDF 的根 link 惯量继续用于物理计算。

控制入口：

- `cmd_vel`：底盘速度。
- `gimbal_cmd`：原有 Yaw／Pitch 的角度目标。
- `lift_cmd`：滑台位移，0～0.16075 m。
- `height_cmd`：按水平机械臂定义的整车高度，0.23925～0.40 m，换算为滑台位移。
- `height_limits`：滑台上下位置反馈；bit 0 下限、bit 1 上限。
- `effort_command`：17 项，前 5 项为时间与底盘目标，后 12 项为原有 10 个力矩及托架／中套筒推力。

升降与 Yaw／Pitch 各自保存目标。双 IMU、关节反馈、洞口接触、真值里程计及 TF 已接入；Odin1 点云已接入 `/red_standard_robot1/odin1/cloud_raw`。

入口为 `make sim`、`make demo`、`make build`、`make test`、`make verify`、`make preview`。构建自动生成扩展 URDF 与 STL。交互运行使用 domain 47，自动测试使用 domain 48。技术栈为 ROS 2 Jazzy、Gazebo Harmonic、gz_ros2_control 和 C++17。

启动文件按 ROS domain 保持单实例；默认显卡渲染 Ogre 2，备用软件渲染 Ogre 1。当前环境的 URDF mimic 测试显示中套筒停在原位，现采用 C++ 半行程同步驱动。

几何检查覆盖 675 组升降、Pitch、Yaw 和舵轮姿态。物理仿真结果见 [验证结果](verification.md)。这些结果对应当前近似模型；真实 CAD 和驱动数据到齐后可以替换调整。

早期版本保存在 `archives/`，其中 `2026-11-15-pitch-only/` 保存本次滑台扩展前的源码、说明与结果，各归档目录带 `COLCON_IGNORE`。

[结构与尺寸](compact_design.md) · [验证结果](verification.md) · [使用说明](../README.md)

迁移时保留依赖与测试结果，原构建产物移入 `.migration_backup/2026-11-15/`，在新路径重新构建。生成目录与本地配置通过 `.gitignore` 排除。

2026-11-15 Odin1 安装：用户最终确认雷达位于炮口左侧，水平朝前。壳体中心在 Yaw 坐标系 `(0.18, 0.13, 0.025)` m，通过固定支架连接 `gimbal_yaw_link`；与原有 Pitch 分成两条连接。雷达随 Yaw 和滑台运动，Pitch 变化时保持相对转盘固定。壳体顶部收纳高度 234.25 mm，整车水平收纳高度仍为 239.25 mm。

雷达采用 240×180、120°×90°、10 Hz 的 Gazebo gpu_lidar 近似，量程设为 0.2～30 m，噪声标准差 0.01 m。扫描原点按壳体前方 24 mm 设置，后续可使用设备 CAD 与标定值调整。RViz 已加入点云显示；纯计算启动使用 EGL。`make verify` 增加已知墙面与地面的雷达场景，检查 Pitch 独立性、Yaw 响应和升降响应。详细说明见 [odin1_mount.md](odin1_mount.md)。

当前 Odin1 版本已完成三组物理仿真的 25 项检查；水平收纳过洞接触数为 0。备用软件渲染将服务端与 GUI 同时设为 Ogre 1，默认显卡纯计算路径使用 Ogre 2 + EGL。

本机软件渲染模式的点云和 TF 功能已验证，Gazebo 结束时出现退出码 -11；默认显卡模式的三组完整测试通过。

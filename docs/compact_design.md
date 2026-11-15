# 竖直滑台与原有 Yaw／Pitch

原始文件保存在 `reference/tunnel_sentinel.urdf`，与用户提供的文件逐字节一致。`scripts/generate_lift.py` 读取它并生成 `urdf/tunnel_sentinel_lift.urdf`。

扩展后的上装连接：

```text
base_link
├── lift_base_joint（固定）→ lift_base_link（外套筒）
├── lift_stage_joint（跟随升降的一半）→ lift_stage_link（中套筒）
└── lift_joint（竖直移动）→ lift_carriage_link（内套筒与托架）
    └── gimbal_yaw_joint → gimbal_yaw_link
        └── gimbal_pitch_joint → gimbal_pitch_link
```

原有连接中，`gimbal_yaw_joint` 的父 link 从 `base_link` 改为 `lift_carriage_link`。Yaw 的原始偏移 `(0,0,0.06525)` m、Pitch 相对 Yaw 的偏移 `(0,-0.11,0.045)` m、全部原有关节类型、轴向及 link 惯性参数继续沿用。`lift_joint=0` 时，原有各 link 的空间位置与原模型一致。

| 项目 | 当前仿真设计 |
|---|---:|
| 滑台总行程 | 160.75 mm |
| 滑台最低、机械臂水平的整车高度 | 239.25 mm |
| 滑台最高、机械臂水平的整车高度 | 400 mm |
| 中套筒行程 | 80.375 mm |
| 外／中套筒长度 | 100 mm |
| 内套筒长度 | 107 mm |
| 三层套筒外宽 | 80／64／48 mm |
| 套筒壁厚 | 4 mm |
| 全伸出时相邻套筒轴向重叠 | 19.625 mm |
| 底盘中央开口 | 92×92 mm |
| 升降目标速度 | 60 mm/s |
| 新增部件估计质量 | 1.30 kg |
| 扩展模型总质量（含 Odin1 与支架） | 55.373409 kg |

套筒尺寸、质量与驱动参数属于测试用工程设定。底盘中央为套筒预留开口；原有底盘质量和惯量继续使用原文件数值。实际结构、传动、刚度与电机数据到齐后，可调整这些参数。

滑台采用一个托架直线关节和一个半行程同步关节。C++ 控制器给托架施加推力，并将中套筒驱动到托架位移的一半。两个推力分别输出，作为套筒传动的仿真近似；滑轮或丝杠的力传递和损耗待实车参数补充。

当前安装组合下，URDF mimic 在物理测试中使中套筒留在原位，因此采用 C++ 同步驱动。配置参考 [gz_ros2_control 的直线关节 effort 接口](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html#simple-setup)。

`lift_cmd` 直接给出升降位移；`height_cmd` 用“水平机械臂时整车高度”换算升降位移。两种指令都保持已有 Yaw／Pitch 目标。`gimbal_cmd` 独立改变原有 Yaw／Pitch，滑台保持自己的位置目标。

40 cm 对应机械臂水平的展开姿态。机械臂向上俯仰时，最高点随俯仰角继续增加；例如滑台完全升起、Pitch=0.52 rad 时，几何最高点约 57.7 cm。通过 25 cm 洞口的准备姿态为 `lift_cmd=0`、`gimbal_cmd.y=0`。

运行时碰撞体使用与 STL 清单一致的盒体和圆柱，保留套筒内孔与底盘开口。几何检查覆盖 675 组升降、Pitch、Yaw 和舵轮姿态；物理测试包含升降独立控制、低姿态过洞和高姿态接触对照。

Odin1 通过固定支架连接 Yaw 转盘，位于炮口左侧并朝前。雷达随升降和 Yaw 运动，与 Pitch 分成两条连接；尺寸与接口见 [odin1_mount.md](odin1_mount.md)。

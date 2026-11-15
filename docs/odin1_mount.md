# Odin1：炮口左侧、朝前、随 Yaw 转盘运动

用户确认：Odin1 位于炮口左侧，水平朝向与炮口一致，安装在可转动云台上。以转盘 +X 为炮口水平朝前、+Y 为左侧。

```text
lift_carriage_link
└── gimbal_yaw_joint → gimbal_yaw_link
    ├── gimbal_pitch_joint → gimbal_pitch_link（枪管俯仰）
    └── odin_mount_joint → odin_mount_link（固定支架）
        └── odin_joint → odin_link（固定壳体）
            └── odin_lidar_joint → odin_lidar_frame（扫描坐标系）
```

Yaw 变化时，雷达与枪管一起转向。滑台变化时，雷达与云台一起升降。Pitch 变化时，雷达相对转盘的位置与姿态保持固定。底盘本身的倾斜也会传到雷达安装架。

## 安装尺寸

| 项目 | 当前模型参数 |
|---|---:|
| 壳体中心相对 Yaw 原点 | x=180、y=130、z=25 mm |
| 壳体方向 | +X 朝前，+Y 朝左，+Z 朝上 |
| 壳体尺寸，前后×左右×上下 | 43×100×62 mm |
| 雷达质量 | 0.28 kg |
| 支架估计质量 | 0.15 kg |
| 仿真扫描原点相对壳体中心 | x=24、y=0、z=0 mm |
| 最低滑台时扫描原点离地 | 203.25 mm |
| 最低滑台时雷达壳体顶部离地 | 234.25 mm |
| 最低滑台、机械臂水平的整车高度 | 239.25 mm |
| 最高滑台、机械臂水平的整车高度 | 400 mm |

支架、壳体外形和扫描原点采用测试近似。真实设备的光学原点、安装孔位、线缆空间和散热间隙，可结合厂家 CAD 与实机标定数据进一步调整。Odin1 的尺寸与视场参考 [官方产品参数](https://store.3dmanifold.com/products/odin1-spatial-memory-module-for-robotics)。厂家还给出了前方近场开阔、四周散热间隙及设备下方空间的建议，见 [官方安装说明](https://github.com/ManifoldTechLtd/wiki/blob/master/docs/odin_series/odin1/3.%20Installation%20Guide_.md)。当前低位安装先服务于 25 cm 过洞仿真，实机阶段需要结合这些要求复核。

参数位于 `src/tunnel_sentinel_description/config/model.yaml` 的 `odin` 部分；支架和外形位于 `meshes/primitives.yaml`。原始 `reference/tunnel_sentinel.urdf` 保留原样，新增内容由扩展生成脚本写入 `urdf/tunnel_sentinel_lift.urdf`。

## 点云

- ROS 话题：`/red_standard_robot1/odin1/cloud_raw`
- 消息：`sensor_msgs/msg/PointCloud2`
- 坐标系：`odin_lidar_frame`，+X 为扫描中心方向。
- 视场：水平 120°、垂直 90°。
- 采样：240×180，每帧 43200 个采样位置，频率 10 Hz。
- 仿真量程：0.2～30 m；距离噪声标准差设为 0.01 m。

点云通过 [Gazebo gpu_lidar](https://gazebosim.org/docs/harmonic/sensors/#lidar-sensor) 与 ROS bridge 产生，采样和噪声为简化设定。颜色图像、设备内部 IMU、MindSLAM 输出及厂商专有字段属于后续接入内容。定位程序应使用随 Yaw 和升降更新的 TF，将每帧雷达数据转换到底盘或地图坐标系。

RViz 配置已加入 Odin1 点云显示。关闭原仿真后，可以运行：

```bash
cd /home/mul/Tunnel_sentry
bash scripts/run_sim.sh rviz:=true
```

默认纯计算入口使用 Ogre 2 + EGL 渲染传感器：`make headless`。备用软件模式 `software_rendering:=true` 将服务端与 GUI 一起切换到 Ogre 1，通过当前桌面显示环境使用 Mesa llvmpipe。

## 检查

`make test` 检查固定连接、雷达跟随 Yaw／升降、Pitch 独立性和 675 组几何姿态。

`make verify` 运行完整演示、洞口接触对照和雷达测距场景。雷达专用场景用已知位置的墙面与地面，比较点云和实际关节位置对应的预期坐标；依次改变 Pitch、Yaw、滑台位置，验证三种运动关系。

单独运行雷达检查：

```bash
cd /home/mul/Tunnel_sentry
source scripts/env.sh
python3 scripts/verify.py --case lidar
```

结果位于 `reports/lidar/`，整体记录见 [verification.md](verification.md)。

本机备用软件渲染检查：Ogre 1 能输出点云，且点云时间戳对应的 `odom` 变换可用；结束进程时 Gazebo 出现退出码 -11。默认 Ogre 2 显卡模式已完成三组仿真检查，日常运行优先使用默认模式。

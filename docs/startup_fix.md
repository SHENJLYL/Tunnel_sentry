> 本页记录滑台扩展前的启动修复；当前机构与测试数据见 [工程状态](project_context.md) 和 [验证结果](verification.md)。

# 2026-11-15 启动问题修复

用户的启动输出显示：`Controller already loaded` 后再次配置控制器失败；图形初始化随后在 Mesa 的 EGL 设备处理中崩溃。

检查发现先前的仿真实例仍在 domain 47 中运行，新启动连接到了已有的控制器服务。旧实例已结束。启动文件现在持有按 ROS domain 区分的文件锁，重复执行启动命令会提示复用已有实例；锁随启动进程退出释放。

图形设置改为默认使用本机显卡与 Ogre 2。实际桌面环境记录为：

```text
GL_VERSION = 4.6.0 NVIDIA 595.99.02
GL_VENDOR = NVIDIA Corporation
GL_RENDERER = NVIDIA GeForce RTX 5070 Ti/PCIe/SSE2
```

备用软件渲染选项 `software_rendering:=true` 使用 Ogre 1 与 Mesa llvmpipe，已完成窗口启动验证。该回退方式参考 [Gazebo 官方排查文档](https://gazebosim.org/docs/harmonic/troubleshooting/)。

完成的实际检查：

- 默认图形启动成功，`joint_state_broadcaster` 和 `sentry_controller` 均为 active。
- 重复启动立即返回已有实例提示。
- 关闭后重新启动成功，两个控制器再次激活。
- `make demo` 在默认图形实例中完成全部 13 项检查。
- 25 cm 洞口通行最高点约 23.984 cm，接触记录为 0。
- 原始 URDF 参数一致性及 243 组姿态检查通过。

结果与日志：

- [演示结果](../reports/latest/result.json)
- [演示轨迹](../reports/latest/trajectory.csv)
- [默认图形启动日志](../log/startup_gpu.log)
- [软件渲染启动日志](../log/startup_software.log)
- [关闭后重启日志](../log/startup_restart.log)
- [完整演示输出](../log/demo_startup_fix.log)

从新场景开始时，在原启动终端按 `Ctrl+C` 并等待退出，然后分别执行 `make sim` 和 `make demo`。

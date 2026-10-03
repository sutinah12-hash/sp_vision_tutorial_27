# Evidence index

当前提交只保留四段能够清楚看到 R1、R2、R3 的最终 RViz 录像，以及构建、动力学和仿真等价性检查。录像均来自 VMware Ubuntu 22.04 的实际 RViz 窗口。

## Final RViz trials

- `final_visible_mpc_rviz_01`：默认 MPC，PASS，96.38 s，终点误差 0.84 cm，跟踪 RMSE 10.60 cm，最小墙距 0.60 m，视频 104.9 s。
- `final_visible_pid_rviz_02`：调优 PID，PASS，161.31 s，终点误差 0.99 cm，跟踪 RMSE 11.06 cm，最小墙距 0.55 m，视频 171.4 s。
- `final_visible_sampling_rviz_01`：Sampling，PASS，97.58 s，终点误差 2.78 cm，跟踪 RMSE 12.65 cm，最小墙距 0.50 m，视频 107.4 s。
- `final_visible_fast_mpc_rviz_01`：Fast MPC，PASS，90.37 s，终点误差 0.91 cm，跟踪 RMSE 11.18 cm，最小墙距 0.55 m，视频 100.0 s。

四轮都是 `trigger=rviz_click`、`goal_messages=1`，使用相同起点 `(0.9, 0.9)`、终点 `(14.1, 14.1)`、地图、三台机器人和仿真参数。默认 MPC、Sampling、Fast MPC 来自干净提交 `3b9ea6c`；PID 来自干净提交 `4208f16`。四轮 `action_status` 均为 4。

青色 R1、绿色 R2、红色 R3 的车体、朝向和标签来自 `/robots` 的只读可视化，不参与规划、控制或碰撞计算。每个试验目录保存 MP4、20 秒实际画面、`metrics.json`、`samples.csv`、参考路径、控制计算时间、启动日志和来源记录。四段 MP4 均已完整解码检查。

`final_visible_comparison/` 保存由四轮 `metrics.json` 重新生成的对比图与 JSON。详细说明见 [RViz controller videos](../video_comparison.md)。

## Other checks

- `final_equivalence/verification.json`：190 组逐格等价对比及受保护文件 SHA-256。
- `dynamics/`：官方动力学库的离线阶跃测量、拟合 CSV、图像与参数。
- `build_tests/`：构建和测试日志；最终复测为 9 个包、21 项检查、0 errors、0 failures、0 skipped。

重新核验证据：

```bash
python3 scripts/validate_evidence.py
```

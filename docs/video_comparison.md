# RViz controller videos

四段最终录像都来自 VMware Ubuntu 22.04 的真实 RViz 窗口。每轮从固定起点 `(0.9, 0.9)` 出发，只点击一次精确目标按钮发布 `(14.1, 14.1)`，随后经过上层行为树、NavigateToPose action、下层行为树、A* 和控制器到达终点。

## Videos

- [Default MPC](evidence/final_visible_mpc_rviz_01/rviz_one_click.mp4)：PASS，96.38 s 到达，终点误差 0.84 cm，跟踪 RMSE 10.60 cm，最小墙距 0.60 m。
- [PID](evidence/final_visible_pid_rviz_02/rviz_one_click.mp4)：PASS，161.31 s 到达，终点误差 0.99 cm，跟踪 RMSE 11.06 cm，最小墙距 0.55 m。
- [Sampling](evidence/final_visible_sampling_rviz_01/rviz_one_click.mp4)：PASS，97.58 s 到达，终点误差 2.78 cm，跟踪 RMSE 12.65 cm，最小墙距 0.50 m。
- [Fast MPC](evidence/final_visible_fast_mpc_rviz_01/rviz_one_click.mp4)：PASS，90.37 s 到达，终点误差 0.91 cm，跟踪 RMSE 11.18 cm，最小墙距 0.55 m。

![Current comparison](evidence/final_visible_comparison/comparison.png)

![Three robots visible in RViz](evidence/final_visible_mpc_rviz_01/video_20s.png)

四段视频都清楚显示青色 R1、绿色 R2 和红色 R3。标记节点直接订阅仿真器已有的 `/robots`，只发布 RViz MarkerArray，不写回仿真器，也不参与规划、代价地图或控制。

## What stayed the same

- 主办方迷宫地图与三台机器人；
- 仿真速度、加速度、频率、碰撞模型和起始位置；
- 全局代价地图参数与主办方机器人参数；
- A*、路径平滑、两层行为树和固定终点；
- 完整主办方 RViz 显示配置。

RViz 保留 Grid、全局/局部 Map、Path、LocalPath、规划 Marker、TF、动态障碍 MarkerArray、PointCloud2 和原控制器 Marker；之后只附加原始 A*、执行轨迹、预测轨迹、三机器人标签、机器人坐标轴、目标箭头和精确目标面板。

## What changed

三组基础方案只切换 `controller` 插件。PID 为适应执行延迟和急弯，使用 `1.0 m/s` 指令上限、`2.0 m/s²` 指令变化限幅、`0.45 m/s²` 制动剖面和 `0.35 m/s²` 横向加速度。Fast MPC 仍使用 MPC 插件，但把学生控制器的速度上限、最大加速度、制动加速度和横向加速度分别设为 `2.0 m/s`、`2.0 m/s²`、`1.2 m/s²`、`1.0 m/s²`。仿真器的 `v_max=2.0 m/s` 与 `a_max=2.0 m/s²` 没有改动。

## How to read the numbers

Action 时间从记录器收到目标开始，到 NavigateToPose 返回成功为止；视频在点击前提前开始，并在成功后保留停车画面，所以视频长度比 Action 时间多约 9–11 秒。

跟踪 RMSE 是实测位置到首次参考路径线段的最近距离。最小墙距是机器人中心到静态占据像素的近似距离；它用于横向比较，不冒充仿真器直接给出的碰撞计数。四组 action 均成功，终点误差小于 3 cm，最终速度接近零。

默认 MPC 在本轮跟踪误差最小，并保留 0.60 m 最小墙距，适合作为主方案。调优 PID 计算最简单且稳定到达，但为提高弯道安全性而主动限速，因此耗时最长。Sampling 可运行但未超过默认 MPC；Fast MPC 最快，同时比默认 MPC 增加了跟踪误差，因此作为速度实验单独展示。

每个目录都保留 `metrics.json`、`samples.csv`、参考路径、计算耗时、启动日志、录像命令元数据、20 秒实际画面和 MP4。PID 来自干净提交 `4208f16`，另外三轮来自 `3b9ea6c`。录像均为 1600×1016、H.264、10 fps、原速连续录制，无拼接和加速，并已逐帧完整解码。

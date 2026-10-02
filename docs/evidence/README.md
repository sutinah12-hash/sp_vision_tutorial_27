# Evidence index

所有图表来自实际记录的数据；RViz 截图和视频来自虚拟机实际窗口。

## Controller comparison

`comparison/summary.json` 和 `comparison/comparison.png` 汇总这 6 轮：

- `final_mpc_01`、`final_mpc_02`：均 PASS。
- `final_pid_01`：FAIL，action 中止后记录至超时；`final_pid_02`：PASS。
- `final_sampling_01`、`final_sampling_02`：均 PASS。

源码版本均为 `52166ed`，工作区干净；固定起终点、相同仿真参数、相同参考路径。每轮目录保存 `provenance.json`、`recorder.log`、`data/metrics.json`、原始 CSV 和数据绘图。失败轮不能从成功率分母删除。

复算：

```bash
python3 scripts/summarize_trials.py docs/evidence/final_mpc_01 docs/evidence/final_mpc_02 docs/evidence/final_pid_01 docs/evidence/final_pid_02 docs/evidence/final_sampling_01 docs/evidence/final_sampling_02 --out results/recomputed_comparison
python3 scripts/validate_evidence.py
```

## GUI and video

[三种控制器录像与计时说明](../video_comparison.md) 汇总同显示条件下的实际结果；不将 GUI 录像混入无界面对照均值。

- `final_sampling_rviz_06/rviz_one_click.mp4`：推荐的橙黄色原始 A* / 绿色平滑路径对照录像。PASS，导航 98.75 s，视频 107.3 s，最终误差 1.38 cm，RMSE 9.68 cm。源码 `8cf53c5` 仅更新 RViz 显示；发目标前重置了 RViz 显示缓存。单次真实点击、未改仿真或控制参数。
- `final_sampling_rviz_02` 至 `_05`：显示排查记录，导航均 PASS，但分别存在原始路径不明显或动态图形刷新异常；`_05` 途中操作显示开关和显示缓存重置。完整过程保留，不作为推荐演示，不混入无界面对照均值。详情见上方计时说明。

- `final_pid_rviz_01/rviz_one_click.mp4`：PID，单次真实点击，PASS；导航 103.70 s，视频 112.6 s，最终误差 1.06 cm，跟踪 RMSE 16.41 cm。
- `final_sampling_rviz_01/rviz_one_click.mp4`：平滑采样实验控制器，单次真实点击，PASS；导航 102.01 s，视频 110.1 s，最终误差 0.56 cm，跟踪 RMSE 10.10 cm。不是完整标准 MPPI。

上述两轮均运行源码 `b4caec9`，其 `src/` 与 MPC 录像源码 `8872af1` 完全相同。每轮保留完整日志、CSV、配置来源和视频抽帧；此前 PID 的失败轮仍然保留。

- `final_mpc_rviz_04/rviz_one_click.mp4`：最终录像，107.5 s，1280×850，10 fps，1075 帧，H.264。实际点击前约 2 s 开始，完整原速录下行驶、action 成功和停车。鼠标事件通过 RViz 按钮发目标。
- `final_mpc_rviz_04/video_before_click.png`、`video_midpoint.png`、`video_arrival.png`：直接解码同一视频的 1 s、50 s 和倒数 2 s 帧。末帧显示 Action SUCCEEDED、ONE goal、误差约 0.012 m、速度约 0.000 m/s。
- `final_mpc_rviz_02`：先前完整成功录像，125.1 s；画面为旧提示面板，未显示实时数值。数值验收由该轮日志和 CSV 支持。
- `final_mpc_rviz_03`：导航成功，新增面板已显示实测数值。录制时工具连接停顿，所得 0.9 s 片段没覆盖行驶过程，因此不把那段片段作为成功录像提交。保留该轮真实导航数据和窗口截图。
- 准备轮 `final_mpc_rviz_01` 未发目标即停止，用于排查 Xwayland 截图方式，不算一次完成导航。

MPC GUI 版本为 `8872af1`，只比对照版本多了 GUI 状态/录屏工具和显示样式；控制器、规划器、仿真器与参数无变化。GUI 成绩不加入无界面对照均值。先完成编译再启动很重要，否则已运行进程可能仍加载旧版动态库。

推荐视频 MP4 已用 `ffprobe` 检查格式、帧数与时长，用 `ffmpeg -i VIDEO -f null -` 完整解码，并目视抽查起始、过程和末尾帧。窗口关闭时 FFmpeg 报失去 X11 窗口并不等同于文件损坏。具体编码参数和开始时间在同名 JSON。

## Other checks

- `final_mpc_raw_01`：无平滑消融，PASS。仅一轮，不代表完整统计结论。
- `final_equivalence/verification.json`：190 组逐格等价对比及受保护文件 SHA-256。
- `dynamics/`：官方动力学库的离线阶跃测量、拟合 CSV/图像/参数；固定朝向辨识不等于完整旋转模型。
- `build_tests/`：9 包编译、GUI 增量编译和最后测试日志。`colcon test-result --verbose` 为 21 checks、0 errors、0 failures、0 skipped；其中也包含包检查，不应全部称为控制器单元测试。

本目录不保存完整开发期所有中间输出；这些留在虚拟机 `results/`。原始测试输出不被覆盖。当前报告范围内的成功、失败与录制问题均在这里说明。

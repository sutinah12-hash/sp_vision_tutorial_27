# Moving obstacle experiments

原先四组录像中，R2、R3 停在初始位置，验证的是静止障碍场景。这里新增四组移动障碍实验，仍使用默认 MPC、PID、Sampling 和 Fast MPC，不修改控制器、地图、仿真动力学、频率、机器人初始位置或 R1 起终点。

## Scenario

四组共用 `two_position_triggered_encounters_v3`。R1 从 `(0.9, 0.9)` 出发，终点仍为 `(14.1, 14.1)`；每轮只由脚本点击一次真实 RViz 固定目标按钮，不直接发布导航目标或速度。

- R3 初始位置 `(9.34, 3.92)`。R1 进入 `x=[9.6,10.8], y=[6.8,7.9]` 时，给 R3 设置 `(8.85,6.15)` 运动目标，在右侧通道形成迎面接近；R3 的 `y>=6.0` 且距 R1 不超过 1.9 m 后，驶回 `(9.4,3.95)`。
- R2 初始位置 `(3.9,7.2)`。R1 进入 `x=[5.15,6.5], y=[5.5,6.7]` 时，给 R2 设置 `(5.8,7.3)` 目标，让它横穿中部转弯路线；R2 的 `x>=5.65` 后，经过 `(4.8,7.55)` 返回 `(3.9,7.2)`。
- 所有指令都通过原版仿真器已有的鼠标右键目标功能输入。机器人由原版运动模型逐步驶向目标，没有瞬移、拖动 R1 或修改物理状态。
- 触发依据是 R1 的位置，不是固定秒数。不同控制器到达触发区的时间不同，但区域、障碍路点、物理模型和触发规则相同。

R2 的回程路点用于避开墙角；R3 的触发区和返回距离门限用于避免慢速控制器还没接近时它就提前离开。正式四轮使用相同的修正场景，预检录像不列入最终对比。

## Evidence

每轮保留两份独立记录：

- `data/`：原有 R1 导航评测，包括唯一目标、action 状态、终点误差、速度、参考路径和实测轨迹。
- `obstacles/`：三台机器人的 TF 位置序列、每次原生右键目标事件、场景配置与动态指标。避免用 `/robots` 中的高斯位置噪声把静止抖动误判成运动。
- `rviz_one_click.mp4`：1600×1016、H.264、10 fps 的原速连续窗口录像。RViz 与仿真操作窗口分开摆放，无遮挡；没有拼接、加速或合成帧。
- `gui_trigger.json`：实际窗口位置、按钮坐标及一次点击记录；`video_validation.json`：完整解码检查和原生会车帧的来源时刻。
- `dynamic_trajectories.png`：由原始 TF 数据绘制的三车轨迹、最近距离及两处会车局部图。图中的线是历史轨迹，不表示三辆车同时占据整条线。

动态场景完成要求：两台障碍车最大位移都超过 1 m；运动时进入首次参考路线 0.35 m 邻域，且当时距 R1 不超过 2 m；按场景完成往返。这个条件用于确认真实运动和路线交互，不替代车体接触检测。

## Read the results

到达 PASS 沿用原评测：一条目标、固定起终点、action 成功，之后等待 4 秒，终点误差小于 5 cm、速度小于 0.08 m/s。它不等同于“无碰撞 PASS”。

三车 TF 先按原版里程计回调生成的相同消息时间戳配对，再以约 10 Hz 采样，并记录消息年龄与共享时间戳。不将新 R1 位置与旧障碍位置直接配对。最近中心距离仍是离散采样结果，不能证明采样间隔内完全无接触。

车体为边长 0.50 m 的旋转正方形。中心距离大于 `sqrt(2)*0.50 ≈ 0.707 m` 是与朝向无关的保守分离条件；小于它时，单凭中心距离无法判定接触与否，也不能直接把差值当成实际车体净空。原版没有在现有话题中提供接触计数，本次不虚构该指标。

静止组沿用此前录像，不是本轮同负载重录。新增三车采样、双窗口和当前虚拟机负载会影响墙钟时间，不能将新旧时间差全部归因于避障。偏离首次参考路径也可能来自合理绕行，移动场景下的 RMSE 不应单独解释为控制精度变差。

本系统的避让链路是“动态占据进入局部代价地图 → 融合到规划地图 → A* 重规划 → 控制器跟踪”，暂不包含对 R2/R3 未来轨迹的显式预测；控制器名称不同不代表都实现了独立的预测碰撞约束。

逐轮视频和数值见 [video comparison](video_comparison.md)，整体对比图见 [dynamic comparison](evidence/final_dynamic_comparison/comparison.png)。

## Reproduce

在 Ubuntu 桌面终端中运行，先结束上一轮仿真：

```bash
cd /home/zrj/nav_final_project
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/test_dynamic_experiment.py
python3 scripts/experiment_desktop.py --save results/desktop_before_dynamic.json
python3 scripts/run_dynamic_trial.py --controller mpc --out results/my_dynamic_mpc_01
python3 scripts/run_dynamic_trial.py --controller pid --out results/my_dynamic_pid_01
python3 scripts/run_dynamic_trial.py --controller sampling --out results/my_dynamic_sampling_01
python3 scripts/run_dynamic_trial.py --controller mpc --fast-mpc --out results/my_dynamic_fast_mpc_01
python3 scripts/experiment_desktop.py --restore results/desktop_before_dynamic.json
```

录像脚本会等待界面就绪并自动点击真实 RViz 按钮，不需要再次点击。运行过程中不要移动鼠标、调整窗口或重新设置目标，以免干扰原生 GUI 输入。每轮默认超时 360 秒，输出目录不能重名。

桌面工具只临时调整 GNOME 单显示器布局，使用 4096×2160 模式供两个窗口分开摆放，不改仿真窗口的 1600×1000 绘制区域或任何物理参数。不支持该模式的电脑需要先手工提供足够的桌面空间；脚本会拒绝录制相互遮挡的窗口。

依赖已有 ROS 2 Humble、NumPy、SciPy、Pillow、Matplotlib、`python3-gi`、`wmctrl`、`xdotool`、`xwd` 和 FFmpeg。轨迹与图表由 `scripts/plot_dynamic_results.py` 复算；录像由 `scripts/check_dynamic_videos.py` 完整解码检查。

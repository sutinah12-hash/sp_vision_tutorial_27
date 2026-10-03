# Controller methods and experiment design

## What does "newer than MPC" mean?

MPC 是一类滚动预测控制方法，而不是一个固定版本的算法。本项目基础实现是使用一阶滞后及延迟模型的凸二次优化跟踪器。MPPI、非线性 MPC、学习辅助 MPC 等仍属于预测控制的大范围，不能简单理解为新名字全面替代旧名字。

工程增加了 `SamplingController`，用于平滑采样预测跟踪实验。该实现借鉴 MPPI 思路，但不是 Nav2 MPPI、SVG-MPPI 或相关论文的完整复现；不使用强化学习训练或 Stein 梯度。

参考原始资料：

- [Nav2 MPPI 官方实现](https://github.com/ros-navigation/navigation2/blob/main/nav2_mppi_controller/README.md)：支持全向底盘的采样式局部控制。
- [Spline-Interpolated MPPI with Stein Variational Inference, 2024](https://arxiv.org/abs/2404.10395)：研究控制序列插值与采样优化。这里只借鉴时间上平滑采样的思路，不含其 SVGD 部分。
- [Smooth MPPI, RA-L 2022](https://arxiv.org/abs/2112.09988)：平滑控制输入的另一条研究路线；本实现不使用该文的输入提升法。

## Common parts: make the comparison meaningful

三个插件共用 `TrackingController` 的路径接收、去重、弧长插值、曲率/制动速度剖面、坐标变换、输出速度限制和加速度限制。默认 MPC 与 Sampling 使用 1.25 m/s 最大指令速度和 1.7 m/s² 指令变化限幅；PID 使用 1.0 m/s 和 2.0 m/s²，并采用更早制动及更低弯道速度。这些是作业控制器参数，不是改动官方仿真器的动力学上限。

所有实验使用相同地图、固定起终点、一次目标消息和同一版本仿真器。实验性采样控制器不会回退为 MPC。

## PID baseline

在路径最近投影点计算横向误差，使用稍前方的路径切向速度前馈，再叠加比例、积分及速度误差反馈。积分限制防止累积过大。距离终点小于 0.70 m 后，切换到无前馈的阻尼位置调节，避免带执行延迟的机器人在终点周围持续振荡。

主办方代价地图参数对齐后，旋转方形底盘在急弯的可用误差余量更小。最终配置把最大指令速度设为 1.0 m/s，制动与横向加速度参数设为 0.45 / 0.35 m/s²，并使用 `kp=2.4`、`ki=0.02`、`kd=0.55`。两轮自动回归和最终 RViz 单击试验均完整到达，最终正式录像用时 161.31 s、终点误差 0.99 cm、跟踪 RMSE 11.06 cm。

## Convex MPC

对 x/y 两个方向使用同一离散模型：

```text
alpha = exp(-dt / tau)
v[k+1] = alpha * v[k] + (1-alpha) * u[k]
p[k+1] = p[k] + dt * v[k+1]
```

参数来自原版动力学库的离线辨识：`tau=0.315 s`，延迟约 `0.21 s`。预测间隔 0.07 s、28 步，覆盖约 1.96 s。最近发出的控制指令保存在队列中，把预测开始的 3 步设为已知固定命令。

代价包含位置误差、速度误差、控制变化和控制幅度；终端位置误差权重更高。凝聚成二次型后，用带速度圆盘投影的加速梯度法求解。每次只执行第一个尚未进入延迟队列的控制量，再根据最新位置重新计算。

这里的速度圆盘约束是 `sqrt(ux²+uy²) <= limit`，不是分别限制 x/y 导致合速度超标。指令最终从世界坐标转换到旋转中的 `base_link`，否则同一条世界方向速度会随底盘姿态跑偏。

## Smooth sampling predictive experiment

对应 `sampling_predictive.hpp`：

1. 使用与 MPC 相同的路径参考、预测时域和延迟队列。
2. 在 6 个时间支撑点产生扰动，再插值成整段平滑扰动序列。
3. 每轮评估 257 条候选序列，正负扰动成对，固定控制器随机种子便于复现。
4. 对每条候选执行速度与指令变化限幅，然后模拟滞后以及加速度上限。
5. 按 `exp(-(cost-min_cost)/temperature)` 加权组合候选，重复 3 轮。
6. 执行第一个自由控制量，下一次根据新状态重新采样优化。

该实现是有限样本的近似优化；**不能保证比凸 MPC 更准确或更快，也不具备论文级全局最优/安全保证**。没有直接评估所有障碍代价，避障路径仍由 A* 与安全平滑提供。是否适合提交为默认算法，必须由当前地图的实测结果决定。

## Metrics and limitations

- 到达：ROS action 成功，目标消息恰好 1 条，起终点正确，停止后的误差小于 0.05 m、速度小于 0.08 m/s。
- 耗时：从收到目标到 action 成功的实际墙钟时间，不用仿真步数冒充真实耗时。
- 跟踪误差：采样位置到首次全局路径线段的最短距离，冻结参考，避免重规划把误差人为变小。
- 计算开销：每次正常控制回调计算的毫秒数，统计平均值、P95 和最大值；这不同于整个系统 CPU 占用率。
- 墙面净空：机器人中心到静态障碍像素的近似距离，不冒充仿真器直接报告的碰撞次数。
- 单次通过不是稳定性证明；实际性能受 VM 调度影响，需要多次运行，并把失败率一起报告。

```bash
python3 scripts/run_trial.py --controller mpc --headless --auto-goal --out results/mpc_new_01
python3 scripts/run_trial.py --controller pid --headless --auto-goal --out results/pid_new_01
python3 scripts/run_trial.py --controller sampling --headless --auto-goal --out results/sampling_new_01
```

`--auto-goal` 只用于自动回归；正式验收使用 RViz 单次点击。

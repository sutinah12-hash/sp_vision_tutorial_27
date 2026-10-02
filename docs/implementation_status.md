# Navigation project implementation status

更新时间：2026-10-02。**这是开发状态记录，不是作业验收报告。**

## 来源与保护范围

- 官方分支：`TongjiSuperPower/sp_vision_tutorial_27:final_project_nav`。
- 本次基线：`05c33fc8aa6695e76bcff310b42f8fdfe276e1cb`。
- 用户旧 `nav` 历史通过合并保留；原来的课程目录和 `main` 不被替换。
- VMware 工作目录：`/home/zrj/nav_final_project`；Ubuntu 22.04、ROS 2 Humble。
- `src/sp_nav_sim`（含预编译动力学库、源码、配置）、原始地图、ControllerPlugin 接口均未修改。
- 不手动发布速度，不拖动起点，不连续发布多个目标。

## 已实现并编译的内容

1. 补齐导航话题、TF 坐标系、参数、插件注册和一键启动。
2. PID 对照控制器：速度前馈、位置反馈、积分限制、速度反馈、速度与加速度限制。
3. MPC：在世界坐标系预测位置和速度，优化有限时域控制量，再转换到旋转的 `base_link`。
4. 从官方动力学库的离线阶跃响应拟合：延迟约 0.2147 s，时间常数约 0.3145 s，速度拟合 RMSE 约 0.00317 m/s。该近似不等于实际导航性能证明。
5. 延迟队列：已发出但尚未执行的命令作为固定约束，优化器返回第一个自由控制量。
6. A* 禁止斜穿障碍拐角；弹性平滑固定端点，并在膨胀代价地图逐段碰撞检查，失败时回退原路径。
7. RViz 配置和固定终点按钮、实际轨迹记录器、单目标自动回归试验脚本。
8. 修复消息包清单的 XML 元素顺序错误；没有通过关闭测试来掩盖问题。

## 验证结果与当前阻塞

- 全部 9 个包编译通过。
- 最近一次 `colcon test-result --verbose` 汇总：12 tests、0 errors、0 failures、0 skipped。其中新增控制器数学测试 7 项。
- 控制器单元测试覆盖坐标旋转、路径去重和制动、空/单点路径、约束、平衡点、闭环收敛、延迟队列。
- **完整导航未通过，尚无可报告的到达精度、全程耗时或 MPC 优于 PID 的结论。**

原始仿真器的 `_on_pub_timer` 同时计算局部代价地图，其周期为 1/30 s。当前虚拟机上，起点附近 58×58 网格的 `_sample_obstacle_mask` 独立测试中位耗时约 38 ms（系统 NumPy 1.21.5），已超过 33.3 ms；实际回调还包含其他工作。线程采样反复落在此函数，`/robots` 实测约 12–13 Hz，`/Odometry` 无数据。两个定时器共用默认互斥回调组，符合过载后里程计定时器饥饿的表现。

已补齐 Python OpenCV，避免 NumPy 慢速距离变换回退。独立目录测试 NumPy 1.24.4 和 1.26.4，单独采样约 29–30 ms，仍未恢复完整运行时的里程计。系统 Python 包没有被这些试验覆盖；相关独立目录不是工程依赖，也不计为成功方案。

课程明确禁止修改仿真器内部源码和参数，因此没有擅自降低频率、改小局部地图、注入改写函数、修改动力学或回调调度。需要管理员确认是否允许纯性能修复，或在更快的环境重试原始仿真器。

## 恢复验证步骤

```bash
cd /home/zrj/nav_final_project
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --base-paths src --executor sequential
colcon test-result --verbose
python3 scripts/benchmark_runtime.py
```

环境问题解决后，使用一个全新的输出目录运行自动单目标回归：

```bash
python3 scripts/run_trial.py --controller mpc --headless --auto-goal \
  --out results/mpc_check_01
python3 scripts/run_trial.py --controller pid --headless --auto-goal \
  --out results/pid_check_01
```

自动发目标只用于回归测试，不替代课程要求的 RViz 单次点击验收。最终在 Ubuntu 桌面终端运行：

```bash
python3 scripts/run_trial.py --controller mpc --out results/mpc_rviz_check_01
```

然后在 RViz 的固定终点面板点击一次。按钮就绪前不点击；不拖拽机器人、不手动发布速度。结束后检查 `data/metrics.json`、`samples.csv` 和 `trajectory.png`。首条全局路径被冻结作为跟踪误差参考，避免重规划从当前位置开始造成误差虚低。

## 最终提交前仍须完成

- 原版环境完整走通；固定起终点、单次 RViz 点击的真实验收。
- MPC/PID 多次公平对照；平滑开/关对照；准确率、时间、路径误差和稳定停车检查。
- 查看真实 RViz 画面并保存实测证据，完成正式 README。
- 重新核对官方最新提交、不可修改文件一致性和全部 ASCII 文件名。
- 验证通过后提交到用户仓库 `nav`，不强推、不提交至官方仓库。

评分标准未给出具体误差/时间对应分值，不能仅凭实现功能承诺 100 分。

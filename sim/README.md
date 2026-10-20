# 可复现闭环仿真

这里生成的是**真实程序运行得到的合成输入仿真结果，不是机器人实测，不是考核成绩**。核心运行器是 C++ `aim_simulator`，链接实车程序使用的同一个 `auto_aim::Controller`、`Target` 和 `Solver`，没有另写 Python 算法替身。

## 数据路径与真值隔离

1. 独立场景生成器创建世界中的装甲板，使用 `Solver::reproject_armor` 和捕获时刻的云台姿态生成图像四角。
2. 加入可固定随机种子的像素噪声、随机缺帧；压力模式还加入 280 ms 连续断流和错误位置观测。
3. 对带噪四角调用真实 `Solver::solve`（PnP）；失败样本直接丢弃，不拿真值补齐。
4. 仅将 `Armor` 观测、图像时间和模拟协议反馈传给 `Controller::step`。真值中心、速度、板号不传给控制器。
5. 执行端具有延迟、有限带宽、角速度和角加速度限制；pitch 按协议向下为正。每个 fire 包对应一发，发射时反馈计数递增。
6. 用 1 ms 时间步积分弹丸运动，与到达时刻每一块独立真实装甲板的有限矩形平面进行碰撞检测。命中不是“预测点与自己一致”。

基准参数：50 Hz 图像、100 Hz 控制步进、20 ms 观测传输延迟、25 ms 执行延迟、35 ms 一阶云台速度响应、22 m/s 弹速。角速度限制 yaw 6 rad/s、pitch 2 rad/s；角加速度限制 yaw 40 rad/s²、pitch 30 rad/s²。相机是无畸变的理想针孔模型；`camera.yaml` **不能用于真实相机**。

## 运行

在项目根目录执行（先按根目录说明构建 `aim_simulator`）：

```bash
./build/aim_simulator --scenario follow --duration 15 --output artifacts/follow
./build/aim_simulator --scenario follow_turn --duration 16 --output artifacts/follow_turn
./build/aim_simulator --scenario low --speed 4 --seed 1 --output artifacts/low4
./build/aim_simulator --scenario medium --speed -7 --seed 7 --output artifacts/medium_minus7
./build/aim_simulator --scenario outpost --speed 2.5 --duration 30 --output artifacts/outpost
./build/aim_simulator --scenario outpost --speed 2.5 --duration 35 --output artifacts/outpost_timeout
./build/aim_simulator --scenario low_to_medium --speed 4 --duration 32 --output artifacts/two_stages
./build/aim_simulator --scenario medium --speed 7 --stress --output artifacts/stress
python3 sim/run_suite.py --binary build/aim_simulator --seeds 1,7,29
python3 sim/run_suite.py --binary build/aim_simulator --seeds 7 --follow-turn-only --output artifacts/follow_turn_suite
./build/aim_simulator --self-test
python3 tests/test_simulator.py --binary build/aim_simulator
```

`--all` 运行单个种子的全部常规场景；`--plot` 将真实控制器输出发给 `tools::Plotter`，字段 `cmd_yaw`、`cmd_pitch`、`estimated_omega` 与独立 `truth_omega` 分开。`--no-assert` 仅用于诊断：即使进程退出 0，报告中的 `passed` 仍保留失败状态，不能据此声称测试通过。

## 场景

以下距离、姿态和速度是可复现的测试输入，不是任务一使用条件或完整能力边界。
尤其 `follow` 的 3 m 和 ±18° 不是作业要求；新增 `task1_pose_tests` 另以独立投影覆盖
0.8 / 1.5 / 3 / 6 m 和多组俯仰、横滚。任务一采用自由姿态 PnP，任务二、三保留整车模型。

- `follow`：3 m 手持目标，视线 yaw 反复跨越 ±π，几何俯仰达到 ±18°，禁止发射。
- `follow_turn`：独立的完整360°跟随检查，目标yaw以 `2π/15 rad/s` 连续转动16s；用实际云台的角增量展开计算总跨度，必须达到 `2π`，不是仅跨±π边界就算转一圈。仍同时覆盖±18°俯仰、丢帧/异常点，禁止发射。与标准58场报告分开保留常规和压力两场报告。
- `low`：四装甲板，3/4/5 rad/s，两个方向；两组半径 0.25/0.27 m、交替高度差 0.10 m。
- `medium`：四装甲板，5/7/9 rad/s，两个方向。
- `outpost`：三装甲板，±2.5 rad/s、半径 0.2765 m；三块相位板的相对高度固定为 `{0, 0.10, 0.20}` m。
- `low_to_medium`：前半段 ±4 rad/s、后半段同方向 ±7 rad/s，旋转相位连续，用于检查每档 10 发和第二档自动识别，不通过丢帧清空额度。

## 输出和验收口径

每次生成 `metrics.json`、逐帧 `telemetry.csv`、逐发 `shots.jsonl`、`tracking.png`、`final.png`。图像由实际采样帧和程序输出渲染；醒目标注 SYNTHETIC INPUT / NOT HARDWARE RESULT，不伪装成 Linux 桌面或相机照片。批量运行额外保留每次命令、标准输出、二进制及源码 SHA-256。

速度报告同时包含 RMSE、95 分位和最大绝对误差。常规场景在最初 2 s 收敛期后统计；档位切换后 2 s、压力断流及随后 1 s 恢复期另外排除出“稳定段”验收，但所有帧误差仍另行保留，不能隐藏恢复过程。低速限值 0.6 rad/s、中速 1.0 rad/s、前哨 0.4 rad/s。合成命中率如实报告，不等同实车命中率，也不伪造文档没有规定的命中率合格线。

自动检查包括数值有限、俯仰限制、过期观测不发射、跟踪覆盖率、发射频率、任务2每档额度、任务1不发射、稳定段速度误差、至少实际执行发射和得到非零合成命中。失败返回非零退出码。

批量测试还要求任务2常规场景确实确认 10 发、两档场景确实确认 20 发，不能仅以“没有超额度”代替完整练习；两档速度分别按低速 0.6 和中速 1.0 rad/s 检查，不用更宽的中速门限替代低速要求。

跟随另设 0.04 rad 的均方根误差工程健全性检查（自定，不冒充文档评分线），完整 15 s 场景还检查实际云台覆盖 ±17° 以上、跨越 yaw ±π 两次以上。独立评分器自检使用静止目标的解析弹道作为正样本、偏航偏移 0.20 rad 作为负样本，并比较 1 ms/0.5 ms 两种积分步长。黑盒测试将同一种子完整跑两次，逐字节比较指标、遥测和截图，验证输出确实可重现。

前哨批量场景实际运行 35 s，用于检查首发后 30 s 停止发射；计分只接受在“首条 fire 脉冲时间 +30 s”之前**实际到达**的碰撞事件，不能把窗口内发出、窗口后到达的弹丸算作有效命中。图表计数也在到达后才增加。这个首发计时约定是当前协议缺少独立考核开始信号时的显式契约，现场应与管理员确认，不冒充官方触发时刻。

## 尚不能替代的实机验证

此仿真从四角开始，不测试 YOLO 的漏检/误检分布；没有真实镜头畸变误差、曝光拖影、USB 相机、串口、IMU 时钟偏移、弹丸阻力、云台间隙与机械共振。因此即使全部合成测试通过，仍需要按真实设备流程检查标定、通信、机械限位、模式切换和现场成绩。不要将本报告当作“全部实机考核已完成”的证据。

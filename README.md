# final_project_aim

同济 SuperPower 自瞄大作业实现，基于官方同名分支模板。
本分支与课程作业所在的 `main` 分支分开；上游模板仍保留历史。

**当前验证范围是 Ubuntu 22.04 构建、自动化测试、离线录像和合成闭环仿真，不是实机考核通过证明。**
演示和日志中的合成结果均标注 `SYNTHETIC INPUT / NOT HARDWARE RESULT`。

## 构建与自检

依赖：CMake、C++17 编译器、OpenCV、Eigen3、yaml-cpp、fmt、spdlog、
nlohmann-json、Python3。实机/录像程序另外依赖海康 MVS、libusb 和
OpenVINO 2024.6；仿真与核心测试可关闭硬件构建，不需要相机或串口。

在本仓库根目录：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DFPA_BUILD_HARDWARE=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

仅构建模拟器：

```bash
cmake -S . -B build-sim -DCMAKE_BUILD_TYPE=Release -DFPA_BUILD_HARDWARE=OFF
cmake --build build-sim -j2
ctest --test-dir build-sim --output-on-failure
```

## 可安全复现的演示

```bash
./build/aim_simulator --scenario follow --duration 15 --output artifacts/follow
./build/aim_simulator --scenario medium --speed 9 --output artifacts/medium9
./build/aim_simulator --scenario outpost --speed 2.5 --duration 35 --output artifacts/outpost
python3 sim/run_suite.py --binary build/aim_simulator
```

每次输出四点投影图、跟踪曲线、指标 JSON、逐帧 CSV 与逐发日志。
完整场景、计分窗口及物理假设见 [仿真运行说明](sim/README.md)。
头部姿态解算与控制器使用同一套生产 C++，不向控制器注入真实板号、真实中心或真实转速。
图像检测类别在仿真中是已知的，因此必须另做真实录像的 YOLO 验证。

已有本地录像时：

```bash
./build/task_1 configs/offline_cpu.yaml --video /absolute/path/video.avi --frames 90 --output artifacts/perception
./build/task_2 configs/offline_cpu.yaml --video /absolute/path/video.avi --frames 90 --output artifacts/perception
./build/task_3 configs/offline_cpu.yaml --video /absolute/path/video.avi --frames 90 --output artifacts/perception
```

可加 `--show` 在已登录图形桌面的终端显示图像，按 Q/Esc 退出。
录像模式使用固定单位 IMU，只验证解码、识别、PnP 与程序流程；不验证动态外参或命中率。
`offline_cpu.yaml` 只是模板 GPU 配置的 CPU 变体，**不是录像或新相机的重新标定**。

## 三个任务入口

- `task_1`：装甲板视线跟随，yaw 跨 ±π 使用短路径、pitch 与指令变化率受限，永不开火。
- `task_2`：车辆 EKF、切板、未来到达位置与弹道求解；中速使用同一估计高度组的正面等待策略。
  每阶段最多 10 个单发命令，5 Hz 上限；确认超时或异常计数锁存禁发，不重试。
- `task_3`：三块不同高度的前哨板，半径 0.2765 m，高度相对初始板估计，过滤顶部板。
  12 Hz 是上限而不是保证射速；首发后 30 秒停止新增发射命令。
- 三任务均向 `tools::Plotter` 的默认本机 UDP 9870 端口输出数值命令角度；
  任务 2/3 额外输出 `estimated_omega`、观测年龄、状态、确认数及阶段。
  可同时检查本地 JSONL，不能仅凭图形窗口判断命中。

## 现场验证前的阻断项

1. 默认不接触硬件。只有显式 `--live` 才打开相机/串口，任务 2/3 另需
   `--allow-fire`。本次开发和测试未执行有效的 live 调用，也未操作真实发射机构。
2. 模板没有摩擦轮启用位或明确的考核阶段事件。当前第二阶段自动识别要求已确认
   第一阶段 10 发，且转速绝对值变化至少 0.8 rad/s 并稳定；**4.4→5.0、5→5、
   +5→-5 等情况不能据此可靠识别**。必须向考核管理员确认接口与流程，
   不得用丢目标或模式抖动重置弹数冒充解决。
3. 30 秒窗口暂以控制器首个单发命令为起点。若考核计时起点另有定义，
   必须先对齐开始信号；仿真只把到达时间在窗口内的碰撞计为命中。
4. 检查真实相机分辨率/内参/畸变、相机安装外参、IMU 时间、轴向符号、
   云台机械限位、弹速、执行延迟、串口字节布局、单包单发和子弹计数语义。
   原模板标定值与默认延迟没有在考核机器人上验证。
5. 当前预测使用无阻力弹道，未标定真实出弹点偏移、空气阻力或机构间隙。
   合成击中有限板面不意味着现场相同命中率；前哨血量不得由理想几何推断。
6. 电控必须独立拒绝非 AUTO 模式命令，并在主机失联时停控停发。
   主机的新鲜度和模式二次检查不能代替电控互锁。按课程要求由负责人监督实机操作，
   不改变考核机器环境，也不超出允许的试射次数。

完整数值证据和截图放在 `validation/`；运行时新产物放在被 Git 忽略的 `artifacts/`。
个人讲解、学习笔记和用户提供的原始参考录屏不在提交内容中。

---

## 原课程仓库说明

# sp_vision_tutorial_27
同济大学SuperPower战队2027赛季算法组招新仓库

Lecture 1:Hello SP Vision  
Lecture 2:Hello C++&&OOP  
Lecture 3:Hello Modern C++  
Lecture4:  
自瞄：Hello Armor&&PnP
导航雷达：Hello ROS2  
Lecture5:   
自瞄：Hello Kalman&&Target
导航雷达：Hello SP Nav


后续课程视频将会全部上传至战队Bilibili官方账号：[TJ-SuperPower战队](https://space.bilibili.com/651837546)

-----------
##### 注意：本仓库main分支为算法组自瞄方向招新使用，nav分支为算法组导航方向招新使用，请各位同学在进行作业提交等操作时注意区分

##### 关于大作业代码：final_project_aim分支为自瞄方向大作业 

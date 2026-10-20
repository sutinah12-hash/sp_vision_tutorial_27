# final_project_aim

同济 SuperPower 自瞄大作业实现，基于官方同名分支模板。
本分支与课程作业所在的 `main` 分支分开；上游模板仍保留历史。

**当前验证范围是 Ubuntu 22.04 构建、自动化测试、离线录像和合成闭环仿真，不是实机考核通过证明。**
演示和日志中的合成结果均标注 `SYNTHETIC INPUT / NOT HARDWARE RESULT`。

本轮构建、窗口尺寸及完整视野验证见 [2026-10-20 验证记录](validation/2026-10-20/README.md)。

## 现场快速开始：编译 → 运行

任务 1、2 的目标使用红色配置。三个程序直接通过终端运行，不额外增加“只观察”步骤。
以下命令一行一条复制，不要复制提示符、额外反斜杠或网页链接格式。

### 第一步：进入项目，准备可执行程序

如果使用配套的预编译包，先将它传到机器人桌面，在 Ubuntu 终端执行：

```bash
cd /home/rm/Desktop
tar -xzf zhaoruijie_aim_ubuntu22_20261020.tar.gz
cd /home/rm/Desktop/zhaoruijie-final_project_aim
bash scripts/check_runtime.sh bin
BIN=./bin
```

若解压目录已存在，先换新的位置，避免覆盖已有修改和日志。
预编译包已含三个任务的 Linux 可执行程序；检查通过后可以跳过重新编译。
它面向 Ubuntu 22.04 x86_64，仍依赖机器人已有的 OpenVINO 2024.6、OpenCV 4.5、
MVS 完整运行环境等，并非任意 Linux 都能直接运行。
包内 `PACKAGE_INFO.json` 记录源码提交、构建环境及校验信息。

若考核需要你逐条编译，或者预编译依赖不兼容，在项目根目录执行：

```bash
cmake -S . -B build-local -DCMAKE_BUILD_TYPE=Release -DFPA_BUILD_HARDWARE=ON -DBUILD_TESTING=ON
cmake --build build-local -j2
ctest --test-dir build-local --output-on-failure
BIN=./build-local
```

也可用 `bash scripts/build_tasks.sh build-local` 完成这三步。
脚本会在存在时加载 `/opt/intel/openvino_2024.6.0/setupvars.sh`，不安装或升级系统软件。
不同电脑应使用新的构建目录，不复制虚拟机的 CMake 缓存。
如果依赖缺失，请让负责人确认已有环境，不替换考核电脑的系统库。
如果你继续使用旧目录 `/home/rm/Desktop/sp_vision_tutorial_27-final_project_aim`，
先把更新的源码放好，再在该目录执行上述编译命令。

### 第二步：一次性选对红色目标配置

让现场负责人确认这辆车使用哪套相机标定。**颜色相同不代表相机标定相同。**

- 确认使用 standard 标定：`CFG=configs/standard_red.yaml`。
- 确认使用 sentry 标定：`CFG=configs/sentry_red.yaml`。

例如，只有确认 standard 标定适用时执行：

```bash
CFG=configs/standard_red.yaml
printf '程序目录=%s\n配置=%s\n' "$BIN" "$CFG"
ls -l /dev/gimbal
lsusb
```

两个新配置都为红色，分别保留原配置的相机内参、畸变、安装外参、旋转、曝光和控制限制。
原来的 `sentry.yaml` 是蓝色；使用它看见红板检测框，不代表控制器会跟随红板。
不要凭电脑名猜标定，也不要为改变敌方颜色而换另一机型标定。
相机、串口应已连接，其他程序不占用设备；现场负责人确认可进行云台测试后再运行。
**新开终端时，要重新进入项目目录，并设置 `BIN`、`CFG`。**

### 任务 1：红色装甲板跟随

1. 完成上面两步后直接运行：

```bash
"$BIN/task_1" "$CFG" --live --show --output artifacts/task1_live
```

2. 按现场规程进入自瞄档位，将红板放入相机视野。正常时能看到闭合四点框和目标标签；
   有效跟随时 `control=1`、`reason=follow_only`，俯仰受限时可能为 `pitch_limited`。
   **任务 1 永不开火**，不需要也不能添加 `--allow-fire`。
3. 现场已有的 Plotter 可接收本机 UDP 9870 的 `cmd_yaw`、`cmd_pitch`（弧度）。
   跟随范围与实际运动效果须在车上核验。
4. 图像窗口获得焦点后按 Q/Esc，或在所属终端 Ctrl+C 停止；等退出日志出现后按现场规程恢复安全档位。
   查看刚才的运行结果：

```bash
python3 tools/summarize_log.py artifacts/task1_live
```

### 任务 2：红色车辆目标跟踪、估速与切板

1. 同样使用已确认的红色 `CFG`，先运行不发射的跟踪程序：

```bash
"$BIN/task_2" "$CFG" --live --show --output artifacts/task2_live
```

2. 在自瞄档位下查看跟随、`estimated_omega`、`omega_variance` 和 `stage`；
   Plotter 同时接收命令角与估计转速。这个命令会控制云台，但不发射。
   任务 2 接受车辆装甲板，不接受前哨或基地类型。
3. Q/Esc 或 Ctrl+C 停止后汇总：

```bash
python3 tools/summarize_log.py artifacts/task2_live
```

4. 正式发射前须先与负责人对齐摩擦轮状态、单包单发和子弹计数语义、低/中速阶段事件。
   当前模板缺少明确阶段事件，自动阶段判别不能可靠覆盖 4.4→5.0、5→5、+5→-5。
   这些接口条件未解决时，只完成不发射测试，不能宣称正式计分流程通过。
   确认完成且现场允许后，用以下命令进入正式测试：

```bash
"$BIN/task_2" "$CFG" --live --allow-fire --show --output artifacts/task2_assessment
```

程序每阶段最多 10 个单发命令、最高 5 Hz。正式测试内不重启程序或人为丢目标来重置额度。
按考核规则核对低速 3–5 rad/s、中速 5–9 rad/s 的估速与命中结果，停止后读取：

```bash
python3 tools/summarize_log.py artifacts/task2_assessment
```

### 任务 3：前哨中部三块装甲板

1. 先确认前哨颜色。**只有前哨也是红色且使用同一标定时**，沿用：

```bash
TASK3_CFG="$CFG"
"$BIN/task_3" "$TASK3_CFG" --live --show --output artifacts/task3_live
```

   若颜色不同，使用正确标定的独立配置修改 `enemy_color`，不要因此更换机型标定。
   此命令会控制云台但不发射。目标须被识别为 `outpost` 并通过中部板姿态检查。
2. 检查旋转方向、约 ±2.5 rad/s 的估计、不同高度板的连续跟踪与 Plotter 输出。
   Q/Esc 或 Ctrl+C 停止后查看：

```bash
python3 tools/summarize_log.py artifacts/task3_live
```

3. 正式发射前与负责人对齐弹速、执行延迟、摩擦轮/弹数接口和 30 秒窗口起点。
   程序当前以首个单发命令为起点；若官方起点不同，须先对齐。
   条件确认且现场允许后运行：

```bash
"$BIN/task_3" "$TASK3_CFG" --live --allow-fire --show --output artifacts/task3_assessment
```

最高 12 Hz，首发后 30 秒停止新增发射命令。实际试射不超出课程允许的 3 次。
仿真中存在窗口伤害仅 600–720、未清除 1500 血量的场景，不承诺实机必定清空前哨。
停止后保存日志供现场核对：

```bash
python3 tools/summarize_log.py artifacts/task3_assessment
```

### 窗口与完整视野

默认窗口请求 960×600，可以拖动缩放。需要更小时，在任一运行命令后加：

```text
--window-size 800x500
```

保持画面比例，比例不同时留空边，不裁切、不拉伸。只改显示窗口，
**不改变送入 YOLO/PnP 的图像大小、关键点坐标或相机内参**。
状态栏在原图上方新增 166 像素画布，不遮住原图顶部；保存 PNG 也保留完整原图区域。
测试逐像素比较原图与加状态栏后的图像区域，检查四边和四角都在。
检测框和目标文字本身仍叠加在图像上。窗口外框的实际大小还受桌面缩放影响。

### 不跟随时看什么

`reason` 保留控制器原始判断，`diagnostic` 解释当前帧，多个问题也可能同时存在：

- `enemy_color_mismatch`：有检测但没有目标颜色，先检查 `enemy=red` 和检测标签。
- `enemy_pnp_rejected`：目标颜色对但 PnP 未通过，检查角点、模糊和实际标定。
- `wrong_armor_type`：目标类型与任务不符。
- `frame_too_old`：采集到决策的时间超过默认 120 ms；检查计算负载与积压，不直接放宽超时。
- `imu_missing` / `telemetry_stale`：姿态或通信反馈不新鲜。
- `mode_not_auto`：串口实际上报的档位不是 AUTO。
- `no_detections`：当前帧没有检测，检查曝光、遮挡和目标位置。

一条命令即可读取指定任务目录里的最新日志，无需拼接长路径或复杂管道：

```bash
python3 tools/summarize_log.py artifacts/task1_live
```

工具打印实际读取的文件路径、原因计数、有效观测帧数及耗时；旧日志缺少的新字段标为未知。
每次运行的 PNG 和 JSONL 在独立的 `artifacts/.../task_N_时间戳/` 目录，保留它们用于复盘。
正常退出发送停控停发命令；断电/进程崩溃时仍依赖电控的独立失联保护。

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

1. 默认不接触硬件。只有显式 `--live` 才打开相机/串口；发射则仅任务 2/3 可另加
   `--allow-fire`。此次虚拟机验证不执行有效 live 调用；
   用户此前已在实机看到图像和检测，但不能将其视为跟随、命中或本次新版实机通过证明。
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

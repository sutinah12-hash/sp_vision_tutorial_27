# Run guide (Ubuntu 22.04 / ROS 2 Humble)

## Existing VMware installation

本次作业使用独立目录 `/home/zrj/nav_final_project`。之前的 `sp_vision_tutorial_27` 和其 `main` 分支不需要删除或覆盖。相机和 AutoDL 不参与这个导航仿真。

打开 **Ubuntu 桌面内的终端**，逐行执行：

```bash
cd /home/zrj/nav_final_project
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/run_trial.py --controller mpc --out results/my_mpc_01
```

等待 RViz 左侧 `One-click navigation` 面板的按钮就绪，再点击一次 `Navigate once: (14.100, 14.100)`。主办方原有的 `2D Goal Pose` 工具没有删除，固定实验使用按钮只是为了让每轮目标坐标完全一致。不要拖动机器人、使用 `2D Pose Estimate`、手动发布速度或重复发目标。程序完成后会停止自己启动的节点，结果保存在 `results/my_mpc_01/data/`。同名目录不可重复使用，这是为防止覆盖上次记录；下一次用 `my_mpc_02`。

主办方 RViz 原有的 Grid、全局/局部 Map、Path、LocalPath、规划 Marker、TF、动态障碍 MarkerArray、PointCloud2 和控制器 Marker 都已保留。附加显示中，橙黄色线是原始 A*，绿色线是平滑参考路径，蓝色线是实测执行轨迹，粉色线是预测轨迹，青色 R1、绿色 R2、红色 R3 是三台机器人的实时车体和朝向，红绿坐标轴是本车姿态，红色箭头是目标。

仿真应同时存在机器人 1、2、3；在启动日志中可看到 `robots=[1, 2, 3]`，RViz 的 `Robots R1-R3` 显示应为 OK 并出现三个标签。代价地图参数与仿真配置一致：机器人半径 0.25 m、余量 0.05 m、安全距离 0.70 m、代价权重 12.0。

`source /opt/ros/humble/setup.bash` 给当前终端添加 ROS 命令/库路径；第二条 `source` 添加本项目编译安装的包和插件路径。启动的新进程会继承这些环境变量，新的终端则需要重新执行。`build/` 是中间编译文件，`install/` 是运行时包布局；二者不提交 Git。

## Compare the controllers

在上一轮退出之后再运行下一轮，不要同时启动多套仿真：

```bash
python3 scripts/run_trial.py --controller pid --out results/my_pid_01
python3 scripts/run_trial.py --controller sampling --out results/my_sampling_01
```

每次运行仍需要在新开的 RViz 中点击一次。`sampling` 是独立的平滑采样预测实验算法，不是完整 Nav2 MPPI 或 SVG-MPPI；没有成功保证，评价应包括失败记录。

自动回归可以加入 `--headless --auto-goal`，它只发送一次目标、不发布速度。正式 RViz 验收不要添加这些参数。关闭平滑做消融实验时添加 `--no-smoothing`。无需修改官方仿真器参数。

## Fresh clone and build

预编译的官方动力学库需要 Ubuntu 22.04 x86_64 和系统 Python 3.10。不要在不匹配的 Conda Python 内编译或运行。

```bash
git clone --branch nav https://github.com/sutinah12-hash/sp_vision_tutorial_27.git nav_final_project
cd nav_final_project
sudo bash scripts/install_dependencies.sh
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --base-paths src --executor sequential
colcon test-result --verbose
python3 scripts/verify_sampling_equivalence.py --out results/my_equivalence_01
python3 scripts/run_trial.py --controller mpc --out results/my_mpc_01
```

若安装依赖需要使用已经允许局域网访问的 Windows 代理，可把已确认可达的代理 URL 作为安装脚本参数。不要把密码、订阅或访问令牌写入代码。现有虚拟机依赖已装好时，不必重复安装。

## Record the actual RViz window

先启动 GUI 试验，但不要马上点击目标。另开一个 Ubuntu 桌面终端：

```bash
cd /home/zrj/nav_final_project
wmctrl -l
```

找到标题含 `RViz` 的那一行，用它的十六进制窗口 ID 替换下面的 `0xWINDOW_ID`：

```bash
python3 scripts/record_rviz.py --window-id 0xWINDOW_ID --out results/my_mpc_01/rviz_one_click.mp4
```

录制开始后，回到 RViz 点击一次固定终点按钮。脚本只读取 RViz 窗口，不修改仿真或发送目标。默认 10 fps、原速、无音频、最长 240 秒，另存 JSON 记录捕获方法。导航试验关闭 RViz 时，FFmpeg 可能报告窗口不再存在；这属于捕获结束，需要用 `ffprobe` 检查 MP4 是否完整，不能当成导航失败。

截图使用 `scripts/capture_rviz.py` 的同名 `--window-id` 参数，输出名改为新的 PNG。它读实际 X11 窗口缓冲区，适用于这里的 Xwayland；不把数据绘图冒充桌面截图。

## Common problems

- `ros2: command not found`：当前终端未加载 ROS 环境，执行两个 `source`。
- 包或插件找不到：确认从本作业目录编译、加载其 `install/setup.bash`，查看 `results/.../launch.log`。
- 没有图形窗口：从 Ubuntu 的桌面终端启动；SSH 本身不等于图形会话。后台测试可以使用 `--headless`。
- 按钮灰色：等地图与里程计启动，检查机器人是否仍在固定起点，是否有其他试验残留；不要通过挪动机器人绕过检查。
- 输出目录存在：改用一个新的试验名称，不要删掉有用证据。
- `cv2` 不存在：安装 `python3-opencv`；`libopencv-dev` 并不保证系统 Python 能 `import cv2`。
- Ctrl+C：脚本只停止自身子进程组，不会批量杀死用户的其他 ROS/VS Code 任务。

完成试验后可以正常使用 Ubuntu 的“关机”。先停止运行中的程序，勿直接强制断电，以免丢失尚未写入的数据。

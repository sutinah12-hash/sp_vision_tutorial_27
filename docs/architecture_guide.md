# Navigation architecture guide

这套工程不是“一个节点算出速度”这么简单，而是把任务决策、路径规划、轨迹跟踪和仿真反馈分成几层。面试时先讲清数据从哪里来、经过谁、最后回到哪里，再讲自己写了什么。

## 1. One navigation cycle

```text
RViz /goal_pose
    |
    v
sp_decision: click_nav.xml                         upper behavior tree
    |
    | NavigateToPose action goal
    v
nav_interface_node: default_nav_with_fallback.xml  lower behavior tree
    |                         |
    | make_plan service       | /set_control_enable
    v                         v
planner_server            controller_server
    |                         ^
    | /global_path            | /Odometry + TF
    v                         |
controller plugin ------------+
    |
    | /sentry/cmd_vel
    v
sp_nav_sim: motion, collision and three robots
    |
    +---- /Odometry, TF, /local_costmap/costmap, /robots
                                             |
                                             v
                                  robot_marker_publisher
                                             |
                                             v
                              /dynamic_obstacles/markers -> RViz
```

它是一个闭环：控制器根据当前位置算速度，仿真器执行速度并产生新的位置，下一次控制周期再用新位置修正指令。只有 `RViz → 行为树 → 规划器 → 控制器 → 仿真器 → 反馈` 全链路接通，机器人才能到达。

## 1.1 从命令到进程：实际启动顺序

运行命令是：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sp_nav_bringup project.launch.py controller:=mpc rviz:=true
```

`project.launch.py` 本身不计算导航，它只组合三个启动动作：

1. 启动 `tf2_ros/static_transform_publisher`，发布 `base_link → livox_frame` 的静态 TF；
2. include `sp_nav_sim/launch/sim_robot.launch.py`，读取 `sp_nav_sim/config/sim_robot.yaml`，启动 `sp_nav_sim` 包的 `sim_robot` 可执行程序；
3. include `sp_nav_bringup/launch/sp_nav.launch.py`，读取 `sp_nav_bringup/config/nav_params.yaml`，启动地图、规划、控制、行为树和 RViz 节点。

ROS 2 launch 会并行拉起这些节点，所以不能把进程启动的先后当成严格的业务顺序。业务上的依赖是：地图先发布代价地图，规划器收到代价地图后才能响应 `make_plan`，控制器收到路径和 TF 后才能计算速度，行为树则负责把这些服务和话题串起来。

`sp_nav.launch.py` 的 `OpaqueFunction(nodes)` 先读取 `controller`、`smoothing` 以及速度/加速度覆盖值，再创建以下节点：

- `esdf_map_publisher`：静态地图和 ESDF/全局代价地图；
- `planner_server`：A* 插件服务器；
- `controller_node`：控制器插件服务器；
- `sp_decision_node`：上层任务行为树；
- `nav_interface_node`：下层 NavigateToPose action 和导航行为树；
- `robot_marker_publisher`、`rviz2`：仅在 `rviz:=true` 时启动，用于显示三台机器人和导航过程。

因此，`controller:=mpc` 并不是启动另一个主程序，而是把 `controller_server.plugin_name/plugin_type` 覆盖为 `MpcController`；`pid` 和 `sampling` 只替换这个插件选择。`mpc_max_speed:=2.0` 之类的实验参数也在这里覆盖 YAML，仿真器的 `v_max` 仍来自另一份 `sim_robot.yaml`。

## 1.2 每个可执行程序的 `main()` 和构造函数

### 仿真器：`sp_nav_sim/sp_nav_sim/sim_robot_node.py`

入口是文件末尾的 `main()`：调用 `rclpy.init()`，构造 `SimRobotNode()`，创建 `MultiThreadedExecutor`，再启动 ROS 回调线程。主线程持续执行 `node.gui.tick(...)`。

`SimRobotNode.__init__()` 的调用顺序是：

1. `super().__init__('sim_robot_node')` 创建 ROS 节点；
2. `declare_parameter()` 声明并读取 `sim_robot.yaml` 中的地图、频率、速度、加速度、初始位置和话题参数；
3. `load_sim_map()` 读取 PGM/YAML 地图；
4. 用主机器人参数构造 `self.sentry = SimRobot(...)`；
5. 根据 `robot_ids/init_xs/init_ys/tf_child_frame_ids` 构造另外两台 `SimRobot`；
6. 创建里程计、TF、局部代价地图、`/robots` 等发布器和 `/sentry/cmd_vel` 订阅器；
7. 创建 `SimGui`。

`SimRobot.step()` 是物理更新入口。它读取最新速度命令，经过速度/加速度限制和碰撞线段检查后，用固定 `dt=1/publish_hz` 更新位置；`SimGui.tick()` 每轮先调用所有机器人的 `step()`，再绘制界面。

### 地图节点：`sp_map_server/src/esdf_map_publisher.cpp`

`main()` 创建 `EsdfMapPublisher` 后 `rclcpp::spin(node)`。构造函数读取 `map_yaml`、`robot_radius`、`margin`、`d_safe` 和 `w`，加载静态地图，创建 `/global_costmap`、`/esdf` 等发布器，并按 `publish_rate_hz` 定时发布。规划器和局部地图节点通过这些话题获得环境信息。

### 规划器：`sp_global_planner/src/main.cpp` 和 `planner_server.cpp`

`main()` 的顺序是 `rclcpp::init()` → `make_shared<PlannerServer>()` → `node->init()` → `rclcpp::spin()`。

`PlannerServer` 构造函数读取 `costmap_topic`、`local_costmap_topic`、`path_topic`、`plugin_name` 和 `plugin_type`，创建代价地图订阅器、`make_plan` 服务、`/goal_pose` 订阅器和路径发布器。`init()` 调用 `loadPlugin()`，由 pluginlib 实例化 `sp_global_planner/AStarPlanner`，随后调用插件的 `configure()`。

收到 `make_plan` 请求后，`onPlanRequest()` 复制当前代价地图，调用 `planner_->createPlan(start, goal)`，发布 `/global_path` 并返回服务结果。`AStarPlanner::createPlan()` 先发布 `/global_path_raw`，再进行安全平滑；平滑结果不安全时返回原始路径。

### 控制器：`sp_controller_server/src/controller_node.cpp` 和 `controller_server.cpp`

`main()` 的顺序是 `rclcpp::init()` → 构造 `ControllerServer` → `node->init()` → `rclcpp::spin()`。

构造函数读取 `/global_path`、`/Odometry`、TF、`/sentry/cmd_vel` 和 `control_frequency`，创建路径/里程计订阅器、速度发布器、`/set_control_enable` 服务和控制定时器。`init()` 通过 pluginlib 加载 YAML 指定的控制器，并调用插件的 `configure()`。

路径到达 `onLocalPath()` 时，服务器调用插件 `setPlan()`；里程计到达 `onOdometry()` 时保存速度并用 TF 把车体系速度转换到 `map` 坐标。50 Hz 的 `controlLoop()` 每次读取最新路径、通过 TF 获取当前位姿，然后调用 `computeVelocityCommands()`，最后将输出转换/发布到 `/sentry/cmd_vel`。控制未使能时，服务器发布零速度。

### 下层导航接口：`sp_nav_bt/src/nav_interface_node.cpp`

`main()` 构造 `NavInterfaceNode`，调用 `init_engine()` 后 spin。构造函数读取 `default_bt_xml` 和 `bt_plugins`，创建 TF buffer/listener。`init_engine()` 加载 `default_nav_with_fallback.xml` 和插件库，再创建 `/navigate_to_pose` action server。

收到 action goal 后，`handle_accepted()` 启动 `execute_tree()` 线程；线程把原始目标写入 BT blackboard，调用 `NavBTEngine::run()`。下层 BT 通常依次执行：获取当前位姿 → 调用 `make_plan` → 设置控制使能 → 等待 `NavStateTrack` 判断到达。成功时调用 action `succeed()`，失败或取消时分别调用 `abort()`/`canceled()`。

### 上层决策：`sp_decision/src/decision_main.cpp`

`main()` 先创建 Qt `QApplication`，再 `rclcpp::init()`、构造 `DecisionNode`。构造函数只检查 `tree_files`、`bt_plugins` 和 `tick_rate_hz` 参数。随后创建 `DecisionEngine` 和 `BlackboardBridge`，加载 `click_nav.xml` 及插件，启动 ROS executor 和 Qt GUI。收到 `/goal_pose` 后，上层 BT 的 `NavToPose` 节点把目标封装成 `/navigate_to_pose` action 请求；它不计算路径，也不发布速度。

## 1.3 一次点击后的消息顺序

1. RViz 固定目标面板发布一次 `/goal_pose`。
2. `BlackboardBridge` 和上层 `click_nav.xml` 接收目标；`NavToPose` 调用 `/navigate_to_pose`。
3. `nav_interface_node` 接收 action，启动下层 BT。
4. 下层 `ComputePathService` 调用 `planner_server/make_plan`。
5. `planner_server` 读取 `/global_costmap`，A* 发布 `/global_path_raw` 和 `/global_path`。
6. `controller_server` 收到 `/global_path`，调用插件 `setPlan()`；同时从 `/Odometry` 和 TF 获取状态。
7. 控制循环调用 `computeVelocityCommands()`，向 `/sentry/cmd_vel` 发布速度。
8. `sim_robot_node` 执行速度、检查碰撞、更新 R1，并发布 `/Odometry`、TF、局部代价地图和 `/robots`。
9. 新的里程计回到控制器，形成闭环；`NavStateTrack` 发现到达后，下层 BT 返回成功，上层 action 返回成功。

### 读代码时的建议顺序

先看 `project.launch.py` 和 `sp_nav.launch.py`，确定节点与参数来源；再看每个包的 `main()`，确认构造函数和 `spin()`；最后沿 `/goal_pose → /navigate_to_pose → make_plan → /global_path → computeVelocityCommands → /sentry/cmd_vel → /Odometry` 搜索回调函数。这样比从某个控制器类直接往外猜调用关系更快。

## 2. What each layer does

### RViz and goal input

RViz 负责显示，不负责规划或控制。`2D Goal Pose` 和左侧的精确目标按钮最终都只发布一条 `/goal_pose`。精确目标按钮固定发布 `(14.1, 14.1)`，便于四组实验使用完全相同的终点；标准 RViz 工具仍然保留。

RViz 配置以主办方文件为基础，保留 Grid、全局代价地图、全局路径、LocalPath、规划标记、TF、局部代价地图、动态障碍、点云和原控制器标记。在这些显示项之后附加原始 A*、执行轨迹、预测轨迹、三机器人标签、机器人坐标轴和目标箭头。

### Upper behavior tree

`sp_decision/decision_trees/click_nav.xml` 是任务层。`BlackboardBridge` 收到 `/goal_pose` 后更新黑板；`ConditionJudge` 判断是否有新目标以及导航是否正在运行；`NavToPose` 把目标封装为 action 请求交给下层导航。完成后再处理底盘和云台状态。

这一层回答的是“什么时候开始导航、当前任务处于什么状态”，不直接算路径和速度。

### Lower behavior tree

`sp_nav_bt/nav_bt/default_nav_with_fallback.xml` 是导航执行层：

1. 以 20 Hz 从 TF 读取 `map → base_link`，得到当前位姿；
2. 以 10 Hz 调用 `make_plan`；
3. 规划成功后调用 `/set_control_enable` 开启控制器；
4. 规划失败时先停控，再重试规划，成功后恢复控制；
5. `NavStateTrack` 计算剩余距离，在 0.04 m 容差内结束 action。

上层行为树面向任务，下层行为树面向导航过程。两层分开以后，将来可以把“点击导航”换成巡逻、追击或补给任务，而规划器和控制器不必重写。

### Map and costmap

`esdf_map_publisher` 读取静态迷宫图，计算每个栅格到障碍物的距离，再生成 `/global_costmap`。本项目导航侧参数与 `sp_nav_sim/config/sim_robot.yaml` 对齐：

- `robot_radius = 0.25 m`：机器人碰撞圆半径；
- `margin = 0.05 m`：额外硬安全边界；
- `d_safe = 0.70 m`：进入该距离后开始产生软代价；
- `w = 12.0`：软代价增长强度；
- `unknown_as_obstacle = false`：与主办方默认值一致。

`robot_radius + margin = 0.30 m` 内为致命区域；0.30 m 到 0.70 m 之间是逐渐升高的代价。这样 A* 不只判断“能不能过”，还会倾向于离墙更远的路线。

### Global planner

`planner_server` 通过 pluginlib 加载 `AStarPlanner`。下层行为树调用 `make_plan` 后，A* 在全局代价地图上搜索，并发布 `/global_path_raw`。开启平滑时，规划器再检查代价与线段安全，输出 `/global_path`；若平滑结果不安全就回退原始路径。

pluginlib 的意义是服务器只依赖统一接口。替换规划算法时，服务器、行为树和话题接口可以不变，只改插件和配置。

### Controller server and plugins

`controller_server` 订阅 `/global_path` 与 `/Odometry`，按 50 Hz 调用当前控制器插件的 `computeVelocityCommands()`，并把结果发布到 `/sentry/cmd_vel`。三种插件共享同一输入输出：

- PID：前馈速度加位置/速度误差反馈；
- MPC：预测带延迟的一阶速度响应，滚动优化一段未来控制量；
- Sampling：对多组平滑控制序列做预测和加权更新。

算法在 `map` 坐标系里计算期望运动，发布前转换到 `base_link`。如果漏掉这一步，机器人转向后，同一组 `x/y` 指令会指向错误的世界方向。

### Simulator and the other robots

`sp_nav_sim` 同时创建机器人 `1、2、3`。机器人 1 是受 `/sentry/cmd_vel` 控制的本车，发布 `/Odometry` 和 `base_link`；机器人 2、3 使用 `base_link_2`、`base_link_3`，作为场景中的其他机器人参与动态障碍和可视化。它们没有被删除。

仿真器把三台机器人的状态一起发布到 `/robots`。`robot_marker_publisher` 只读这个话题，再向 `/dynamic_obstacles/markers` 发布车体、朝向箭头和 `R1/R2/R3` 标签。它没有控制输出，也不写代价地图，所以增加或关闭该显示不会改变轨迹。颜色分别为 R1 青色、R2 绿色、R3 红色。

仿真器仍使用主办方参数：`v_max=2.0 m/s`、`a_max=2.0 m/s²`、30 Hz 仿真与发布频率、相同地图和初始位置。代码中的批量栅格计算与回调分组只解决虚拟机调度过慢，不改变这些参数、机器人数量、碰撞模型或消息接口。

## 3. How to read an RViz run

先看左侧状态是否为 OK，再按下面顺序观察：

1. 全局代价地图是否覆盖静态迷宫；
2. 是否能看到 R1、R2、R3 三个彩色车体，TF 中是否有 `base_link`、`base_link_2`、`base_link_3`；
3. 点击目标后是否出现原始 A* 与平滑路径；
4. 实际轨迹是否贴近参考路径且没有穿入高代价区；
5. 预测轨迹是否随机器人向前滚动；
6. 到达后 action 是否成功、速度是否接近零。

不要只看“最后到了”。若路径正确但机器人撞墙，问题多半在代价地图尺寸、控制误差或坐标转换；若根本没有路径，优先检查目标、TF、地图和规划服务；若有路径但不动，检查控制器使能、里程计和 `/sentry/cmd_vel`。

## 4. Questions likely to be asked

### Why use two behavior trees?

上层组织任务状态，下层组织一次导航的规划、重试、控制使能和到达判断。任务逻辑变化时，不需要改底层导航模块。

### Why are planning and control plugins?

服务器保持话题、服务和生命周期不变，算法通过统一接口替换，便于比较 A*、其他规划器以及 PID、MPC、Sampling。

### Why does the costmap use the robot radius?

路径搜索的是机器人中心。如果不按半径膨胀，中心能通过的格子不代表机器人外形能通过，转弯时尤其容易擦墙。

### What are the hard and soft safety regions?

0.30 m 内是半径加余量形成的致命区；0.30–0.70 m 是软代价区，用于让规划器主动远离墙面，而不是只在碰撞前最后一刻避开。

### Why does the controller need odometry and TF?

里程计提供位置、速度和朝向，TF 统一 `map` 与 `base_link` 坐标。控制器必须知道自己在哪里、运动多快以及车体朝向，才能闭环修正。

### Why not send the whole predicted sequence at once?

MPC 每周期只执行第一步，然后用新的测量重新预测。这样可以不断消除模型误差、调度延迟和扰动带来的偏差。

### Why is PID faster to compute but less accurate here?

PID 只根据当前误差反馈，计算量小；它不显式预测惯性和延迟，在连续弯道容易产生更大的横向误差。MPC 计算稍多，但能提前考虑未来参考与执行延迟。

### What does the fallback in the lower tree do?

规划失败时先关闭控制器并输出零速，再按限定次数重试；重新获得有效路径后才恢复控制，避免沿旧路径盲走。

### How do the other two robots enter the system?

仿真器从 `robot_ids: "2,3"` 创建它们，并把三台机器人放进 `/robots`，同时发布对应 TF。只读标记节点把 `/robots` 转成 RViz MarkerArray；局部地图则直接使用仿真状态构建动态占据。显示支路和避障支路相互独立。

### Which result should be recommended?

默认 MPC 是主方案，因为它在到达、跟踪误差和安全余量之间更均衡。快速 MPC 更快，但通常会增加跟踪误差，因此作为单独的速度实验，不替代稳健基线。具体数值以 README 中最终四轮对照为准。

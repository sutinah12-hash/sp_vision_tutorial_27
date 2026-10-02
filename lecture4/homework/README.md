# nav_lecture4小作业：Qos_debugger
这道题的目标就是让你快速上手、理解什么是ros。抛开复杂的概念，ros本质上完成的任务就是便利的进程间通信。比如，我有两个进程，一个进程发布雷达数据，另一个进程接收。使用ros就可以方便的完成通讯。你可以搜索以下，发送信息有哪些类型，分别有何特点。特别注意，不同的信息传输方式有不同的质量要求。你不会允许送的外卖没到你手上，但是一个电话过来，也许漏接了也无所谓，可能只是个诈骗。ros2也是这样。重点关注这一点会对这道题有所帮助

> 环境要求：ROS2 Humble

## 包结构

```
src/
  nav_hw_interfaces/     # 接口包：只放 .msg，无业务代码
    msg/SensorData.msg   # 可以打开.msg文件查看接口详细内容
  qos_debugger/          # 业务节点包
    src/qos_debugger_pub.cpp   # 发布 /SensorData
    src/qos_debugger_sub.cpp   # 订阅 /SensorData
```

## 编译

```bash
cd lecture4/homework
colcon build 
source install/setup.bash
```


## 任务一：实现pub和sub的通信

```bash
ros2 -h     //有忘记的命令就输入-h去查询用法
```

**现象**：启动pub和sub节点后sub节点订阅不到任何消息
提示：如果两个节点不能通过话题通信，我们应该如何区查看话题的详细信息（有没有相关的命令）
任务一仅修复qos_debugger_pub.cpp的一处或几处代码即可完成


---

## 任务二：为什么收到的消息会丢包？/(ㄒoㄒ)/~~

第一问找到问题并修改代码后，记得重新
```colcon build```
```source install/setup.bash```
**现象**：sub会打印黄色的warning输出告诉你丢包的序列，每秒还会打印出丢包率

提示：
有没有什么命令可以查看节点的配置(ros2 param -h)
可以通过修复qos_debugger_sub.cpp中的一处或几处代码解决该问题（可能会有多种解决方法）

## 任务三：把收到的消息的帧率计算并打印出来（放在定时器回调函数中每秒打印一次即可）
补全qos_debugger_sub.cpp即可



在下面按顺序完成三个任务，要求把用到的命令放入代码块中并讲解命令，每一问最好加入自己的理解



## 作业解答

### 任务一：恢复 pub/sub 通信

先用下面的命令确认话题是否存在，并查看发布端与订阅端声明的 QoS：

```bash
ros2 topic list
ros2 topic info /sensor_data --verbose
```

`ros2 topic list` 列出当前话题；带 `--verbose` 的 `ros2 topic info` 会显示每个
endpoint 的 Reliability、Durability 和队列深度，适合定位“话题存在但收不到消息”。

原代码中 publisher 提供 `best_effort`，subscriber 请求 `reliable`。发布端无法满足
订阅端要求，DDS 会判定二者不兼容。我把 publisher 的默认 Reliability 改为
`reliable`，这样两端的 QoS 兼容，subscriber 可以正常接收消息。

```bash
ros2 run qos_debugger qos_debugger_sub
ros2 run qos_debugger qos_debugger_pub
```

### 任务二：消除默认配置下的丢包并正确统计

使用参数命令检查订阅回调的人为延迟：

```bash
ros2 param list /sensor_subscriber
ros2 param get /sensor_subscriber callback_delay_ms
ros2 param set /sensor_subscriber callback_delay_ms 0
```

publisher 默认以 100 Hz 发布，也就是每 10 ms 一条；原 subscriber 每条消息故意休眠
30 ms，理论处理上限只有约 33 Hz。单线程执行器来不及消费，KeepLast(10) 队列很快被
新数据覆盖。我将默认延迟改为 0 ms，使正常配置可以跟上 100 Hz 输入；同时在发现序号
跳变时执行 `lost_count_ += lost`，让累计丢包数和丢包率与 warning 一致。

下面的命令可以故意恢复 30 ms 延迟，复现丢包并验证统计逻辑：

```bash
ros2 run qos_debugger qos_debugger_sub --ros-args -p callback_delay_ms:=30
```

### 任务三：计算实际接收帧率

定时器每秒读取一次累计接收数。我保存上次的计数和 `steady_clock` 时间点，用
“本周期新增消息数 / 实际经过秒数”计算 FPS。使用实际时间间隔而不是固定除以 1，
可以避免定时器调度误差导致帧率偏差。日志每秒同时输出累计丢包率和最近一秒的帧率。

### 完整编译与验证

```bash
cd lecture4/homework
source /opt/ros/humble/setup.bash
colcon build --packages-select nav_hw_interfaces qos_debugger
source install/setup.bash
```

先启动 subscriber，再在另一个终端启动 publisher；正常配置应稳定接收约 100 Hz，
且累计丢包率保持 0%。

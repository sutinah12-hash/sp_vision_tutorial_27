# Lecture4 所学总结：ROS2 通信、QoS、参数与帧率统计

## 1. 作业目标

本次作业使用两个 ROS2 节点：

```text
sensor_publisher -- /sensor_data --> sensor_subscriber
```

需要完成：

1. 修复 pub/sub 无法通信的问题；
2. 找到默认配置下丢包的原因并正确统计；
3. 每秒计算并打印实际接收帧率；
4. 在 README 中记录命令和自己的理解。

## 2. 任务一：QoS 兼容性

发布者最初提供 `best_effort`，订阅者请求 `reliable`。订阅者要求的服务质量高于发布者能够提供的服务质量，因此 DDS 判断二者不兼容。

发布者默认可靠性修改为：

```cpp
this->declare_parameter("reliability", "reliable");
```

排查命令：

```bash
ros2 topic list
ros2 topic info /sensor_data --verbose
```

`--verbose` 可以查看各 endpoint 的 Reliability、Durability 和队列深度。

[查看真实 QoS endpoint 输出](docs/evidence/zrj_lecture4_qos_info.txt)

## 3. 任务二：丢包原因与修复

发布者默认 100Hz，每10ms发布一条消息。订阅者原来每次回调睡眠30ms，理论最大处理速度只有：

```text
1 / 0.03s = 33.3Hz
```

生产速度长期高于消费速度，`KeepLast(10)` 队列很快被新消息覆盖。因此正常配置把默认回调延迟改为0ms。

发现序号跳变时执行：

```cpp
const uint32_t lost = msg->seq - expected_seq_;
lost_count_ += lost;
```

这样 warning、累计丢包数和丢包率保持一致。

参数排查命令：

```bash
ros2 param list /sensor_subscriber
ros2 param get /sensor_subscriber callback_delay_ms
ros2 param set /sensor_subscriber callback_delay_ms 0
```

## 4. 任务三：实际 FPS

定时器不一定每次恰好间隔1秒，因此正确公式是：

```text
FPS = 本窗口新增接收数 / 实际经过秒数
```

代码使用 `std::chrono::steady_clock`，因为它单调递增，不受系统校时影响。每次报告后同时更新上次接收数和上次时间。

## 5. 实际验证

正常模式：

- Publisher 与 Subscriber 均为 `RELIABLE`；
- `callback_delay_ms=0`；
- 丢包 warning 为0；
- 累计丢包率为0%；
- 稳定阶段达到约100Hz。

[查看正常模式真实日志摘要](docs/evidence/zrj_lecture4_normal.txt)

压力模式命令：

```bash
ros2 run qos_debugger qos_debugger_sub --ros-args -p callback_delay_ms:=30
```

本次压力验证成功复现179次丢包 warning，接收速度约33Hz，最后记录收到193条、丢失363条、丢包率65.29%。压力测试用于证明检测逻辑有效，不是正常提交配置。

[查看压力模式真实日志摘要](docs/evidence/zrj_lecture4_stress.txt)

一键重新验证：

```bash
cd lecture4/homework
bash zrj验证.sh
```

## 6. 我的收获

- 话题存在不代表节点一定能通信，QoS 仍可能不兼容；
- `depth` 只能缓冲短暂突发，不能解决消费者长期慢于生产者；
- Reliable 保证传输策略，但应用层回调长期处理不过来仍会积压；
- 当前使用单线程执行器；若换成多线程执行器，计数器需要互斥锁或原子变量；
- ROS2 调试应同时检查 topic、QoS、parameter、序号和真实时间窗口。


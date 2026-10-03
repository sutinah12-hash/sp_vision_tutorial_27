# Implementation and validation history

正式使用说明见根目录 README 和 [run guide](run_guide.md)。本页记录开发与验证过程。

## Protected scope and authorization

- 官方来源：`TongjiSuperPower/sp_vision_tutorial_27:final_project_nav`，基线 `05c33fc8aa6695e76bcff310b42f8fdfe276e1cb`。
- 独立 VMware 工作目录：`/home/zrj/nav_final_project`，Ubuntu 22.04 / ROS 2 Humble。
- 旧 `nav` 历史通过合并保留；此前课程目录及 `main` 不覆盖。
- 仿真器优化仅涉及 `sim_robot_node.py` 的采样与回调分组，仿真参数保持不变，详见 [optimization record](simulator_optimization.md)。
- 动力学库、运动与碰撞实现、地图、仿真 YAML/launch、控制器接口均与官方基线逐字节一致；哈希及逐格比较结果见 `evidence/final_equivalence/verification.json`。
- 不发布人工速度、不拖动起点、不二次更换目标；正式 GUI 试验一次点击，自动回归明确另行标注。

## Resolved issues

1. 修复空的话题/坐标系/路径参数，补齐 launch 与 pluginlib 注册名称。
2. 修复消息包 manifest 的 XML 元素顺序，保留测试检查而非跳过。
3. 安装系统 `python3-opencv`，确保仿真 Python 进程可以加载 `cv2`。
4. 原始逐格 Python 地图计算超出单周期预算，同组里程计回调饥饿。经授权后向量化采样并隔离状态/命令回调组，恢复完整导航。
5. 早期离散线段检测会漏掉很短的障碍角穿越；新增测试暴露后，改为精确线段与障碍格矩形相交检测。
6. PID 使用横向误差、切向速度前馈和终点阻尼调节；按主办方代价地图重新配置速度、制动、弯道速度与增益后，两轮自动回归和最终 GUI 试验均完整到达。
7. 恢复主办方完整 RViz 配置，保留所有原显示项，再追加原始 A*、执行轨迹、预测轨迹、机器人坐标轴、目标和精确单击面板。
8. 将导航代价地图的机器人半径、余量、安全距离、代价权重和未知区域策略与 `sim_robot.yaml` 对齐。

## Evidence stages

- `a232215`：控制器、平滑、启动、记录工具初版。
- `92814a3`：官方动力学辨识、延迟模型、回归测试。
- `62f0a0e` / `60e11bb`：分别记录获准的采样等价优化与回调隔离。
- `7efc9af` / `2e61a38`：终点 PID 调节、规划单测与精确安全检查。
- `1de1f82`：独立平滑采样预测实验控制器。
- `52166ed`：本轮重复对照的代码版本，增加原始计算计时与试验 provenance。
- `8872af1`：增加真实状态面板及窗口录屏工具；控制、规划与仿真源码未改。GUI 录像最终验收使用该版本。
- `3b9ea6c`：增加 R1、R2、R3 只读 RViz 标记，三台机器人在录像中清晰可见。
- `4208f16`：按主办方代价地图调节 PID，并增加可复现实验参数覆盖；正式 PID 录像使用该版本。

## Validation boundaries

9 个包编译通过，`colcon test-result` 汇总 21 项检查，0 errors、0 failures、0 skipped。覆盖控制器数学、采样收敛、坐标转换和 A* 平滑安全性；这不等于全部实际场景的安全证明。

真实导航的逐轮结果、计时、轨迹和起终点检查保存在 `docs/evidence/`。三机器人清晰显示后的四轮 GUI 汇总位于 `evidence/final_visible_comparison/summary.json`。PID 在正式录像前另完成两轮自动单目标回归；最终提交只保留四段 GUI 验收证据。

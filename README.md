# final_project_aim

同济 SuperPower 课程大作业，提交分支为 `final_project_aim`，不修改课程作业所在的 `main`。

**当前不是任务二、三的修复交付。** 本轮只精简说明并核验离线日志、图像显示测试；没有修改自动瞄准、发射或安全限制，也没有连接机器人。
历史完整回归为 22/24，通过记录不能代替实机验收；两个中速扰动案例仍失败。
详细结果保留在 [上次验证记录](validation/2026-10-20-pnpfix/README.md)。

以下是 Windows → Ubuntu 22.04 虚拟机 → VS Code 终端的**离线验证**流程，不启动相机、串口、云台或发射。
每个代码块按顺序执行；某一步报错就先停止，不继续粘贴后续命令。

## 1. Windows PowerShell：上传已有压缩包

这个文件是已有的任务一定位修复包，**不是本轮新生成的任务二、三修复包**。
先确认虚拟机 IP 仍为 `192.168.30.129`，再执行：

```powershell
Test-Path -LiteralPath 'D:\Users\zhaoruijie\Downloads\zhaoruijie_aim_ubuntu22_20261020_pnpfix.tar.gz'
ssh zrj@192.168.30.129 'mkdir -p /home/zrj/Downloads'
scp 'D:\Users\zhaoruijie\Downloads\zhaoruijie_aim_ubuntu22_20261020_pnpfix.tar.gz' zrj@192.168.30.129:/home/zrj/Downloads/
```

第一条应输出 `True`；SSH/SCP 要求密码时输入虚拟机账户密码，输入过程不显示字符。

## 2. Ubuntu 虚拟机终端：解压到新目录

```bash
cd /home/zrj/Downloads
mkdir zhaoruijie_offline_check
tar -xzf zhaoruijie_aim_ubuntu22_20261020_pnpfix.tar.gz -C zhaoruijie_offline_check
cd /home/zrj/Downloads/zhaoruijie_offline_check/zhaoruijie-final_project_aim
sha256sum -c MANIFEST.sha256
code .
```

`mkdir` 若提示目录已存在，停止并换一个新目录名，后面的路径也要一起修改；不要覆盖已有源码和日志。
校验结果应全部为 `OK`。若 `code` 命令不存在，在 VS Code 的“文件 → 打开文件夹”中选择上述项目目录。
压缩包中的说明来自旧提交；以 GitHub 本分支的 README 为当前说明。

## 3. VS Code：打开终端并编译离线检查

选择菜单“终端 → 新建终端”，依次执行：

```bash
cd /home/zrj/Downloads/zhaoruijie_offline_check/zhaoruijie-final_project_aim
cmake -S . -B build-offline-check -DCMAKE_BUILD_TYPE=Release -DFPA_BUILD_HARDWARE=OFF -DBUILD_TESTING=ON
cmake --build build-offline-check --target task_diagnostics_tests -j2
ctest --test-dir build-offline-check --output-on-failure -R '^(task_diagnostics_tests|log_summary_tests)$'
```

2026-10-23 已在现有虚拟机源码上验证上述编译、测试命令：`2/2` 通过。这里只检查显示、窗口参数、诊断信息和日志工具，**不是三个任务的完整验收**。
显示测试逐像素检查原图和四边四角都保留；这不等于相机标定或实车跟随已经正确。

需要的现有开发环境：CMake、C++17 编译器、OpenCV、Eigen3、yaml-cpp、fmt、spdlog、nlohmann-json、Python3。
关闭硬件构建后不需要 MVS 或 OpenVINO。若依赖缺失，保留报错；不要在考核电脑上自行替换系统库。
不要复制另一台电脑的 CMake 构建目录；不同电脑重新配置、编译。

## 4. 查看已有日志

在上述项目根目录执行，读取随源码保存的历史离线日志：

```bash
python3 tools/summarize_log.py validation/2026-10-20-pnpfix/video_telemetry.jsonl
```

查看自己的旧运行日志时，将最后一个参数替换成实际的 `telemetry.jsonl` 文件或包含它的目录。
传目录时工具选择其中最新的日志，并打印实际读取路径。日志中的命令不证明机器人执行成功或命中。

## 仍未解决

- 任务二：两个中速扰动回归失败；阶段识别和回执处理仍有已记录的问题。
- 任务三：计时起点与现场流程尚未对齐，既有仿真也不代表全部考核目标完成。
- 本 README 不包含实车自动瞄准、发射启动或减弱安全检查的操作流程。

验证证据保留在 `validation/`，运行产物写入 Git 忽略的 `artifacts/`；个人讲解不随本分支提交。

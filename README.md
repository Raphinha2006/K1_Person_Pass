# K1 独立视觉向人传球

本项目从真机修改前基线 `ade91251cd3156691040db39c17f1667c709f6e6` 提取依赖。
原目录 `Booster_K1_3v3_Demo` 和原版副本 `AAA` 保持不变。
已在 K1 上进行基础交互测试，用户现场确认具备找人和踢球能力；落点、距离和稳定性留待后续优化。

当前功能分支改为直接调用原生视觉踢球技能，尚未完成此版本的实机踢球验收。

快速启动见 [简短运行说明](RUN.md)，后续控制调整思路见 [日志分析笔记](docs/PASS_CONTROL_NOTES.md)。

运行环境为 Booster K1、ROS 2 Humble 及机器人现有 SDK。相机标定和 TensorRT 模型沿用本机配置。

## 保留的内容

- 原 Vision、K1 相机标定及当前配置使用的两个 TensorRT 模型，未修改视觉算法。
  修复原 Vision 关闭录制时访问空日志对象的启动崩溃。
- `booster_interface`、`booster_msgs`、`vision_interface` 三个原有接口包。
- `src/interface/brain` 仅保留基线原样的 `brain/msg/Kick` 消息定义，匹配固件 `/kick_ball`；不包含 Brain 节点。
- 一个新增 C++ 控制包 `src/person_pass`。

没有 Brain、裁判机接收程序、比赛行为树、队友通信或球场全局定位。
Vision 输出的 `position_projection` 已在机器人坐标系中；置信度单位沿用原接口的 0–100。
短期缓存的人球位置通过本机里程计转换，避免机器人转动后继续使用旧方向。
保留里程计不代表需要足球场定位；本程序不订阅球场标记或比赛状态。

## 控制流程

等待机器人状态 → 准备模式/起身 → 行走或足球模式反馈 → 站稳 0.5 秒
→ 找到最近接球人和球 → 直接请求原生 `VisualKick(start=true, version=1)`
→ 持续提供球和接球人位置 → 本轮结束或中断时退出技能并停车。

- 起身使用原 `kGetUp` 接口，配置 `getup_version=1` 选择已在本机验证的 BMM 起身；
  `0` 为基础起身实现。版本字段依据[官方接口](https://docs.booster.tech/zh-CN/docs/developer-guide/cpp/rpc/motion/)。
  不会在正在起身或正在跌倒时切换模式。
- 起身重试仅在原版的 `planner=1` 且报告倒地时发出；`planner=10/23` 表示两种原生起身过程，
  即使恢复状态暂时仍为倒地，也不会再次发起或切换模式。
- 恢复状态沿用原三字节反馈，站稳要求 `recovery=0`、`planner=8` 或 `20`，与原版一致。
  `planner` 是原版使用的固件状态索引，不能与 SDK 的 `RobotMode` 数值混用。
- 未站稳时不发非零速度；模式转换最多等待 30 秒，起身最多请求两次。
- 真机恢复状态实测约 1 Hz，使用独立的 `recovery_timeout=2.5` 秒；头部和里程计使用 0.8 秒超时。
- 站稳确认阶段退出旧技能；本轮启动一次原生技能，运行中不再发送行走速度与它争抢控制。
- 绕过原 demo 的比赛状态、角色、球场位置、距离、角度和连续对准门槛。
  固件内部的平衡、接近球和出脚条件仍由原生技能处理，并不保证请求后立即抬脚。
- 版本线值为 `0=V1`、`1=V2`；已修正先前误用的 `2`。本机原生踢球反馈 planner 为 `12/22`，
  技能过程中不会把它们误判成未就绪并切回准备模式。
- 每帧从有效 `Person` 中，按机器人坐标系的平面距离选择最近的一人；其他人仍参与近距离避障。
- 人或球过期、状态丢失、近距离人/对手机器人遮挡时停车。
  视野暂时没有目标时只转头搜索，不盲目走动。
- `/kick_ball` 的球坐标来自视觉，`dir` 为球指向人的角度，`goal_x/y` 为人的机器人坐标。
  不使用原 demo 的球门位置；`robot_theta_to_field` 以本地里程计方向为参考。
- `kick_power=1.5` 是交给固件的期望距离参数，真实滚动距离尚未标定。
  `kick_duration=7.0` 是技能本轮最长运行时间，不是机器人行走时间，也不是一次踢腿的完成反馈。
- 目标/状态失效、遮挡、取消、退出程序或到时，显式发送 `VisualKick(start=false)` 并停车。
  本轮不自动重试；停止再启动才会开始下一次。

主要新增代码：`controller.hpp` 是状态机；`person_pass_node.cpp` 负责 ROS 输入、SDK 请求和状态发布。
测试 `controller_test.cpp` 不连接机器人；没有新增 Python 控制程序。
`adapter_test.cpp` 在 ROS domain 83 中使用模拟输入，动作请求重映射到
`/person_pass_test/requests`，分别验证预览拦截和请求序列；`/kick_ball` 也重映射到隔离测试话题，不连接真机动作通道。

## 编译与检查

```bash
./scripts/build.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select person_pass vision --event-handlers console_direct+
colcon test-result --verbose
```

两项 `person_pass` 检查使用模拟检测，不能替代真实 Vision 启动检查。
`vision_no_recording_startup` 在 K1 上实际加载 GPU 模型，并以 `save_data=false` 启动和退出 Vision；
使用隔离的 ROS domain 83，不启动传球控制节点。合计应为三项检查。

## 启动

默认预览模式：

```bash
./scripts/start.sh
```

`dry_run=true` 会拦截全部运动、起身、转头、踢球和模式切换请求，只发布到 `/person_pass/preview`；
踢球目标只发布到 `/person_pass/kick_preview`，不会提供给固件。
预览模式不会模拟机器人模式反馈，因此没有真实就绪反馈时会停在初始化阶段。

明确启用真机动作：

```bash
./scripts/start.sh --real
```

脚本发现其他 demo 节点运行时会拒绝启动，不会结束其他 demo。
Ctrl+C 只停止本脚本启动的两个进程，控制节点退出前发送零速度。

状态、停止和重新触发：

```bash
ros2 topic echo /person_pass/status
ros2 service call /person_pass/enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /person_pass/enable std_srvs/srv/SetBool '{data: true}'
```

参数在 `config/person_pass.yaml`。技能运行时由固件控制动作和速度，旧推球控制的速度限制参数已移除。
启动参数为只读，修改配置后需要重启；运行中启停使用上述服务。
日志中的 `VISUAL_KICK_REQUEST start=true version=1` 表示已发出技能请求；
仍应结合固件反馈的 planner 和现场踢腿动作确认执行，不能把请求本身当作踢球成功。

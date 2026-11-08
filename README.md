# K1 视觉向人传球

在 Booster K1 上，通过相机找球和接球人，追球、调整位置并调用原生踢球技能向人传球。不需要裁判机、队友通信或球场定位。

## 当前功能

- 自动处理准备、起身和就绪检查；启动过程会请求足球模式。
- 选择视野内最近的人作为接球人，通过本机里程计补偿缓存位置。
- 丢失目标后六点转头扫描，一轮约6秒；仍未找到则原地转身并继续扫描。一次固定找一个目标，短暂重见目标不会无条件重置扫描进度。
- 人的位置记忆4秒、球2秒；人记忆有效时可用于追球和绕球。出球前要求球在1秒内、人在2.5秒内有有效观测。
- 球到人距离至少 **0.5米**，满足现有接近及朝向条件后启动原生踢球。动作最长7秒，冷却2秒后继续下一轮。
- 停用、反馈过期或倒地时停止动作；搜索和接近期间遇到近距离遮挡停车。原生踢球期间不发送行走指令争抢控制，短暂丢失目标不会中断本次动作。

## 复用了哪些原始 demo 接口

依赖提取自真机原始基线 `ade9125`（AAA）；本项目独立运行，不加载 AAA 的完整 Brain 或比赛行为树。

| 来源 / 接口 | 用途 |
| --- | --- |
| 原 Vision、`vision_interface`，`/booster_soccer/detection` | 获取球和人的类别、置信度及机器人坐标；沿用相机标定和模型 |
| `booster_interface`、`booster_msgs`，`/LocoApiTopicReq`、`/LocoApiTopicResp` | 使用原 SDK 请求转头、移动、起身、模式查询/切换和 `VisualKick` |
| `/fall_down_recovery_state`、`/low_state`、`/odometer_state` | 获取恢复状态、头部反馈和本机里程计 |
| 原样 `brain/msg/Kick`，`/kick_ball` | 向固件提供球坐标、接球目标、方向和力度 |
| AAA 的头部扫描动作序列和找球思路 | 移植六点扫描、按最后目标方向转身的逻辑，不加载比赛定位流程 |

原 Vision 另修复了关闭录制时访问空日志对象的启动崩溃。这里的 `brain` 包仅提供 Kick 消息，不运行原 Brain 节点。

## 自己设计的部分

新增 C++ `person_pass` 控制节点，负责独立状态机、最近接球人选择、人位置滤波与跳变过滤、人球记忆、分步搜索、追球绕球及原生技能接管。

- **比例力度控制**：`power = clamp(1.2 × 球到人的距离, 1, 10)`，启动动作时计算一次。系数可配置，实际距离受地面和固件影响。
- **实时目标更新**：动作中每50毫秒更新 `/kick_ball` 的球位置、目标和方向。
- **搜索进度保留**：未恢复有效接近决策前，不因短暂看到目标反复从头扫描。
- **日志和测试**：发布状态，记录力度、估计球程和技能响应；包含控制逻辑与隔离 ROS 接口测试。

主要源码：`src/person_pass/include/person_pass/controller.hpp`（决策）、`search.hpp`（搜索）、`src/person_pass/src/person_pass_node.cpp`（ROS 与 SDK 适配）。

## 怎么运行

环境：Booster K1、ROS 2 Humble、机器人现有 SDK 和匹配本机的 TensorRT 模型及标定。

首次下载（模型和部分库使用 Git LFS）：

```bash
git lfs install
git clone https://github.com/Raphinha2006/K1_Person_Pass.git
cd K1_Person_Pass
git lfs pull
./scripts/build.sh
```

在当前真机上启动：

```bash
cd /home/booster/Workspace/K1_Person_Pass
./scripts/start.sh --real
```

**启动脚本就是 `scripts/start.sh --real`**，同时启动 Vision 和传球节点，自动循环运行。按 **Ctrl+C** 停止。若提示 `Already running`，先退出其他 demo。

不加 `--real` 为预览模式，拦截运动及踢球目标输出，不驱动机器人；它不模拟就绪反馈。

参数在 `config/person_pass.yaml`，修改后重启。日志在 `logs/person_pass.log` 和 `logs/vision.log`。改源码后重新运行 `./scripts/build.sh`。

可选检查：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select person_pass
colcon test-result --verbose
ros2 topic echo /person_pass/status
```

已在真机完成交互测试，现场确认具备找人、找球和向人传球能力。固件返回成功代表接受技能请求，实际传球方向和落点仍以现场观察为准，后续可继续优化精度。

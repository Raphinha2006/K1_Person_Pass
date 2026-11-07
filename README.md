# K1 独立视觉向人传球

本项目从真机修改前基线 `ade91251cd3156691040db39c17f1667c709f6e6` 提取依赖。
原目录 `Booster_K1_3v3_Demo` 和原版副本 `AAA` 保持不变。
当前版本供代码审查，尚未做实机动作验收，也未推送 GitHub。

## 保留的内容

- 原 Vision 源码、K1 相机标定及当前配置使用的两个 TensorRT 模型，未修改视觉算法。
- `booster_interface`、`booster_msgs`、`vision_interface` 三个原有接口包。
- 一个新增 C++ 控制包 `src/person_pass`。

没有 Brain、裁判机接收程序、比赛行为树、队友通信或球场全局定位。
Vision 输出的 `position_projection` 已在机器人坐标系中；置信度单位沿用原接口的 0–100。
短期缓存的人球位置通过本机里程计转换，避免机器人转动后继续使用旧方向。
保留里程计不代表需要足球场定位；本程序不订阅球场标记或比赛状态。

## 控制流程

等待机器人状态 → 准备模式/起身 → 行走或足球模式反馈 → 站稳 0.5 秒
→ 找到唯一接球人和球 → 接近球 → 绕球调整方向 → 连续对准 0.35 秒
→ 单次推踢 → 停车。

- 起身使用原 `kGetUp` 接口；不会在正在起身或正在跌倒时切换模式。
- 恢复状态沿用原三字节反馈，站稳要求 `recovery=0`、`planner=8` 或 `20`，与原版一致。
  `planner` 是原版使用的固件状态索引，不能与 SDK 的 `RobotMode` 数值混用。
- 未站稳时不发非零速度；模式转换最多等待 30 秒，起身最多请求两次。
- 站稳确认阶段退出旧的自动视觉射门，后续由本节点显式发送速度控制。
- 人或球过期、多人、状态丢失、近距离人/对手机器人遮挡时停车。
  视野暂时没有目标时只转头搜索，不盲目走动。
- 出球沿用原 `cross` 的行走推踢思路，不调用自动视觉射门。力度、落点需要后续真机标定。
- 出球结束或中断后不自动重试。先停用再启用才会开始下一次。

主要新增代码：`controller.hpp` 是状态机与几何控制；`person_pass_node.cpp` 负责 ROS 输入、SDK 请求和状态发布。
测试 `controller_test.cpp` 不连接机器人；没有新增 Python 控制程序。
`adapter_test.cpp` 在 ROS domain 83 中使用模拟输入，动作请求重映射到
`/person_pass_test/requests`，分别验证预览拦截和请求序列，不连接真机动作通道。

## 编译与检查

```bash
./scripts/build.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select person_pass --event-handlers console_direct+
colcon test-result --verbose
```

## 审核后启动

默认预览模式：

```bash
./scripts/start.sh
```

`dry_run=true` 会拦截全部运动、起身、转头和模式切换请求，只发布到 `/person_pass/preview`。
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

参数在 `config/person_pass.yaml`。默认速度上限为前进 0.4 m/s、侧移 0.3 m/s、转向 0.6 rad/s；
出球速度参数 0.35 m/s。最低有效速度沿用原 demo 的限制方式，实际行走表现仍需验证。
启动参数为只读，修改配置后需要重启；运行中启停使用上述服务。

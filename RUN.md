# 简短运行说明

在 K1 的终端执行：

```bash
cd /home/booster/Workspace/K1_Person_Pass
./scripts/start.sh --real
```

程序自动处理起身、切入足球模式，看到球和接球人后先追球、绕到球后对准，再启动原生踢球技能；没有识别到人时只等待，不会盲踢。
每轮技能最多运行 7 秒，结束后冷却 2 秒并自动重新找球，循环直到手动停止。
力度按球到人的距离正比给出（现场标定：5 m → 6 档，`power = 1.2 × 距离`，限制 1–10）。

- **停止**：在启动终端按 `Ctrl+C`。
- **只预览、不动作**：运行 `./scripts/start.sh`，不加 `--real`。
- **首次或改源码后编译**：在项目目录运行 `./scripts/build.sh`。
- 若提示 `Already running`，先退出其他 demo，再启动本程序。

参数：`config/person_pass.yaml`。日志：`logs/person_pass.log`、`logs/vision.log`。
每轮日志输出 `KICK_POWER distance=… power=…` 与 `KICK_TRAVEL target=… measured=…`，可用于继续校准力度。

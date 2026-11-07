# 简短运行说明

在 K1 的终端执行：

```bash
cd /home/booster/Workspace/K1_Person_Pass
./scripts/start.sh --real
```

程序自动处理起身，选择离机器人最近的人，追球、调整并尝试一次传球。无需裁判机。

- **停止**：在启动终端按 `Ctrl+C`。
- **再传一次**：停止后，重新运行上面的启动命令。
- **只预览、不动作**：运行 `./scripts/start.sh`，不加 `--real`。
- **首次或改源码后编译**：在项目目录运行 `./scripts/build.sh`。
- 若提示 `Already running`，先退出其他 demo，再启动本程序。

参数：`config/person_pass.yaml`。日志：`logs/person_pass.log`、`logs/vision.log`。

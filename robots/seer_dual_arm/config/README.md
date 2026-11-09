# Seer 双臂配置

从仓库根目录运行 `rpent --robot seer_dual_arm` 时，机器人默认读取此目录。

| 文件 | 用途 |
| --- | --- |
| `robot.json` | `teleop.homing.initial_poses.left/right` 为两个机械臂的初始末端位姿；同时配置到位阈值。 |
| `robokit.json` | SDK 的 DDS 地址、domain 和 namespace，真机模式使用。 |
| `api.toml` | API planner 默认读取的模型、地址和认证信息；仅保存在本机。 |
| `api.example.toml` | 不含凭据的 API 配置模板，可提交到 Git。 |

`pos` 是 `base_link` 坐标系下的 `[x, y, z]`，单位米；`quat` 是 `[x, y, z, w]` 四元数。
当前数值来自本机 SeerTeleoperationService 的归位配置。修改初始位时，先运行预览，确认后再用 `--live`。
归位只控制两个机械臂末端，两个夹爪保持原状。

首次配置 API 时复制模板并填写中转站信息：

```bash
cp robots/seer_dual_arm/config/api.example.toml robots/seer_dual_arm/config/api.toml
chmod 600 robots/seer_dual_arm/config/api.toml
```

标准 bearer 认证可直接填写 `api_key`，或通过 `env_key` 指定凭据环境变量。
自定义请求头认证使用 `[http_headers]`，例如 `x-bf-vk`。`api.toml` 已被此目录的 `.gitignore` 忽略，也不会打入安装包。

安装环境后，从仓库根目录启动预览对话：

```bash
./start_seer_chat.sh
```

输入“回到初始位置”触发一次预览，再输入后续消息可继续对话；`/exit` 退出。
脚本顶部的 `LIVE=false` 表示预览，改为 `LIVE=true` 表示真机模式；`UV_BIN` 默认是控制器的 `/opt/uv/uv`。
脚本会切换到项目目录，并将额外参数传给 RPent，例如 `./start_seer_chat.sh --output-dir logs/my-chat`。
`--robot-config`、`--robokit-config` 和 `--api-config` 可分别覆盖文件路径。
显式使用 `--service-root` 时，机器人配置取自对应外部目录；API 配置仍单独选择。

完整安装、Claude 对话及真机前提见[使用指南](../../../docs/source-zh/rst_source/real_world_robots/seer_dual_arm.rst)。

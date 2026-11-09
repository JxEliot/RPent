# Seer 双臂配置

从仓库根目录运行 `rpent --robot seer_dual_arm` 时，机器人默认读取此目录。

| 文件 | 用途 |
| --- | --- |
| `robot.json` | `teleop.homing` 配置初始位；`teleop.cartesian` 配置末端增量；`teleop.gripper` 配置夹爪动作参数和超时。 |
| `robokit.json` | SDK 的 DDS 地址、domain 和 namespace，真机模式使用。 |
| `api.toml` | API planner 默认读取的模型、地址和认证信息；仅保存在本机。 |
| `api.example.toml` | 不含凭据的 API 配置模板，可提交到 Git。 |

`pos` 是 `base_link` 坐标系下的 `[x, y, z]`，单位米；`quat` 是 `[x, y, z, w]` 四元数。
当前数值来自本机 SeerTeleoperationService 的归位配置。修改初始位时，先运行预览，确认后再用 `--live`。
归位只控制两个机械臂末端，两个夹爪保持原状。

末端工具与 `dual_franka` 使用相同的 `move_delta`、`rotate_delta` 名称，坐标系为 Seer 的 `base_link`：

- `move_delta(arm, delta_xyz)`：`arm` 为 `left`、`right` 或 `both`，增量 `[dx, dy, dz]` 单位米。`both` 用一条轨迹同步移动双臂，保持各自姿态。
- `rotate_delta(arm, delta_rpy)`：单臂 `left` 或 `right`，外禀 XYZ 欧拉角增量单位弧度；`q_target = q_delta * q_current`，保持位置。

单臂动作会将另一臂保持在动作开始时的实测位姿；两个夹爪均保持原状。
`teleop.cartesian` 的默认值如下，旧配置缺少此节时也使用这些默认值：

```json
"cartesian": {
  "max_translation_m": 0.10,
  "max_rotation_rad": 0.5,
  "position_threshold_m": 0.005,
  "rotation_threshold_deg": 1.0,
  "stable_cycles": 3
}
```

上限按单次增量向量的欧氏范数检查，超限会报错；这些上限不代表工作空间或碰撞检查。
到位需左右臂连续满足阈值的不同反馈样本达到 `stable_cycles` 次。
预览只报告请求的增量，不连接硬件，也不生成实测起点或绝对目标。
例如输入“以 base_link 的 +Z 为上，双臂同时向上移动 10 厘米”，对应
`move_delta("both", [0, 0, 0.10])`；“右臂绕 base_link 的 Z 轴正向旋转 0.1 弧度”对应
`rotate_delta("right", [0, 0, 0.1])`。

夹爪通过单独启动的 service 控制。真机对话中可输入“张开左夹爪”“关闭双夹爪”“将右夹爪设为 50”或“查看夹爪状态”。
位置 0 为闭合，100 为张开；速度、加速度、减速度、力和超时在 `teleop.gripper` 中配置。
参考 service 启动时会打开两个夹爪。双侧命令分别发送，不保证同步；状态没有请求 ID，超时或部分发送时请检查实际状态。

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

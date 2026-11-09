Seer 双臂机器人
================

此集成支持末端增量控制，以及将两个机械臂末端返回到 Seer
``robots/seer_dual_arm/config/robot.json`` 中 ``teleop.homing.initial_poses`` 配置的位置。它使用机器人自带的
``seer-robokit-py`` 包，以及 ``SeerTeleoperationService/return_to_initial.sh``
所使用的 WBC 命令和末端位姿反馈 topic。动作在仓库内的
``robots/seer_dual_arm/homing.py`` 和 ``cartesian.py`` 中实现，直接调用 Python SDK，
不会执行外部 shell 脚本或导入原服务；配置直接存放在此集成目录内。
两个夹爪保持原状，无需启动 VLA、相机或遥操作服务。

安装
----

为控制器 SDK 创建独立的 Python 3.12 环境：

.. code-block:: bash

   uv venv --python 3.12
   source .venv/bin/activate
   uv pip install -e ".[seer-dual-arm]"

此 extra 安装 ``seer-robokit-py==0.1.6``，与参考的 Seer 服务使用版本一致。RPent
自身的基础依赖仍然需要安装；此集成不需要 VLA、仿真器、相机或夹爪服务 extra。
以下命令使用 ``uv run --no-sync`` 运行此环境，避免解析仓库内其他机器人的可选依赖。

配置文件
--------

编辑 ``robots/seer_dual_arm/config/`` 中的文件：

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - 文件
     - 用途
   * - ``robot.json``
     - ``teleop.homing`` 配置初始末端位姿和归位阈值；``teleop.cartesian`` 配置增量上限和到位阈值。
   * - ``robokit.json``
     - 真机模式所用的 SDK DDS 地址、domain 和 namespace。
   * - ``api.toml``
     - 模型、接口地址与凭据；文件存在时，API planner 默认读取它。
   * - ``api.example.toml``
     - 可提交到仓库、不含凭据的 API 配置模板。

初始位和 DDS 参数与参考的本机 Seer 服务一致。``pos`` 是 ``base_link`` 坐标系下的
``[x, y, z]``，单位米；``quat`` 是 ``[x, y, z, w]`` 四元数。使用默认配置无需原服务目录。

首次使用时，复制 API 模板并填写中转站配置：

.. code-block:: bash

   cp robots/seer_dual_arm/config/api.example.toml robots/seer_dual_arm/config/api.toml
   chmod 600 robots/seer_dual_arm/config/api.toml

填写 ``model``、``base_url``、``wire_api``（``responses`` 或 ``chat``）。标准 bearer 认证
可直接填写 ``api_key``，或通过 ``env_key`` 指定凭据环境变量（默认 ``OPENAI_API_KEY``）。
自定义请求头认证使用 ``[http_headers]``，例如 ``x-bf-vk = "YOUR_RELAY_KEY"``。
``api.toml`` 已加入 Git 忽略，也不会打入安装包。
可通过 ``--robot-config``、``--robokit-config``、``--api-config`` 分别覆盖文件路径。

通过中转站与 GPT-6 Astra 对话
-----------------------------

先检查连接，再启动预览对话：

.. code-block:: bash

   uv run --no-sync rpent-check-llm --planner api \
     --api-config robots/seer_dual_arm/config/api.toml

   uv run --no-sync rpent \
     --robot seer_dual_arm --planner api \
     --reasoning-effort low --tui

在控制器上，也可直接运行 ``./start_seer_chat.sh``，脚本使用 ``/opt/uv/uv`` 启动上述对话。
将脚本顶部的 ``LIVE=false`` 改为 ``LIVE=true`` 即可启用真机模式；
``UV_BIN`` 和 ``REASONING_EFFORT`` 也在顶部配置。
脚本会切换到项目目录，并透传其他参数，例如 ``./start_seer_chat.sh --output-dir logs/my-chat``。

终端会等待第一条输入，收到消息后才请求模型。输入“回到初始位置”即可让模型调用工具。
预览模式只显示目标，模型会说明机器人未移动。每次回复后可以继续输入，使用 ``/exit`` 退出。
检查预览并清空工作区后，在同一对话启动命令后添加 ``--live``，重新启动即可执行真机动作。
启动对话本身不会移动机器人，用户的动作指令才会触发工具。
SDK 在首次真机动作时初始化，并保持连接直到对话结束。每次动作都会等待新的实测位姿反馈；
共享反馈 reader 在会话退出时由 SDK 统一释放。

可通过 ``--model openai:gpt-6-astra`` 覆盖文件中配置的其他模型。
Responses 接口对应 ``openai:<model>``，Chat Completions 对应 ``openai-chat:<model>``。
也可通过 ``--api-config ~/.codex/config.toml``（别名 ``--api-codex-config``）
显式复用已有 Codex 自定义 provider。它直接读取地址、模型与请求头等配置，
不会启动 Codex，也不读取 ``auth.json`` 中的 OAuth 登录信息。
中转站除了文本生成，还必须支持 function calling。

与 Claude 对话
--------------

按 :doc:`../guides/configure_planner` 配置 ``ANTHROPIC_API_KEY``；
使用兼容的中转站时，再配置 ``ANTHROPIC_BASE_URL``。然后运行：

.. code-block:: bash

   uv run --no-sync rpent \
     --robot seer_dual_arm --planner claude_code \
     --model sonnet --tui

在空白的 ``you>`` 提示符后输入“回到初始位置”。启动时添加 ``--live`` 可启用真机动作。
此集成只向 Claude 提供末端运动和结束会话工具，默认关闭内置 shell 与文件工具。
使用 ``/quit`` 退出。Claude、API 和 direct planner 共用同一套内部运动实现。

末端增量控制
------------

接口名称参考 ``dual_franka``，坐标系使用 Seer 共同的 ``base_link``：

* ``move_delta(arm, delta_xyz)``：按 ``[dx, dy, dz]`` 平移，单位米，保持姿态。
  ``arm`` 可选 ``left``、``right`` 或 ``both``。``both`` 使用一条 WBC 轨迹，
  让双臂按相同进度移动同样的增量；只有双臂实测都到位才报告成功。
* ``rotate_delta(arm, delta_rpy)``：选择 ``left`` 或 ``right``，绕 ``base_link``
  坐标轴旋转，保持位置。增量为外禀 XYZ 欧拉角，单位弧度：
  ``q_target = q_delta * q_current``。

单臂动作会将另一臂保持在动作开始时的实测位姿。所有动作保持夹爪原状，复用已有 SDK 会话。
真机增量的起点来自新收到的实测反馈，与配置的初始位无关。
预览只报告请求的增量，不生成实测起点或绝对目标。

在对话中，以 ``base_link`` 的 +Z 为“向上”，输入“双臂同时向上移动 10 厘米”，
模型会调用一次 ``move_delta("both", [0, 0, 0.10])``。
旋转示例：“右臂绕 base_link 的 Z 轴正向旋转 0.1 弧度”。也可直接预览：

.. code-block:: bash

   uv run --no-sync rpent \
     --robot seer_dual_arm --planner direct \
     --direct-tool move_delta \
     --direct-tool-args '{"arm":"both","delta_xyz":[0,0,0.10]}'

``robot.json`` 中可选的 ``teleop.cartesian`` 使用以下默认值；旧配置缺少此节时也使用这些值：

.. code-block:: json

   {
     "max_translation_m": 0.10,
     "max_rotation_rad": 0.5,
     "position_threshold_m": 0.005,
     "rotation_threshold_deg": 1.0,
     "stable_cycles": 3
   }

每次增量向量的欧氏范数必须在配置上限内，超限会报错。这些上限不提供工作空间或碰撞检查。
到位需左右臂连续满足末端控制阈值的不同反馈样本达到 ``stable_cycles`` 次。
轨迹复用归位的 ``--duration``、``--timeout``、``--feedback-timeout``、``--hz`` 参数。

预览
----

默认运行会读取回初始位配置并显示目标，不会导入或初始化 ``robokit``：

.. code-block:: bash

   uv run --no-sync rpent \
     --robot seer_dual_arm \
     --planner direct \
     --direct-tool return_to_initial

默认使用此集成的 ``config/robot.json`` 和 ``config/robokit.json``。
显式传入 ``--service-root`` 时，改用外部服务目录的 ``config/rbk_config.json``
和 ``config/controller/robokit.json``。``--rbk-config`` 保留为 ``--robot-config`` 的别名。机器人配置必须包含
左右臂的 ``initial_poses``；如果定义了 ``teleop.homing.enabled``，其值必须为 true。

真机动作
--------

先检查预览并清空机械臂工作区。真机控制需要交互式终端，并显式添加 ``--live``：

.. code-block:: bash

   uv run --no-sync rpent \
     --robot seer_dual_arm \
     --planner direct \
     --direct-tool return_to_initial \
     --live

控制器必须在 ``/RobotControl/WbcDebugTF`` 发布实测变换。动作会读取左右臂新鲜的末端反馈，
通过 smoothstep 位置插值和最短路径四元数插值向 ``/RobotControl/WbcEECommand`` 发布命令，
并等待位置和姿态误差连续满足配置阈值 ``stable_cycles`` 次。默认轨迹时长为 5 秒、超时为
30 秒、等待反馈 5 秒、命令频率 20 Hz；可通过 ``--duration``、``--timeout``、
``--feedback-timeout`` 和 ``--hz`` 调整。动作中断时，如果仍能读到新鲜反馈，会发布当前位置作为保持目标。

此集成只控制机械臂末端，不会控制夹爪，也不会运行 VLA。正式操作前仍需在已连接的 Seer 控制器上验证实际动作。

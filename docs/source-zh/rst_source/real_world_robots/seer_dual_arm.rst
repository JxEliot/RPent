Seer 双臂机器人
================

此集成提供一个固定动作：将两个机械臂末端返回到 Seer
``rbk_config.json`` 中 ``teleop.homing.initial_poses`` 配置的位置。它使用机器人自带的
``seer-robokit-py`` 包，以及 ``SeerTeleoperationService/return_to_initial.sh``
所使用的 WBC 命令和末端位姿反馈 topic。它不会启动模型、相机、遥操作或夹爪服务；两个夹爪保持原状。

安装
----

为控制器 SDK 创建独立的 Python 3.12 环境：

.. code-block:: bash

   uv venv --python 3.12
   source .venv/bin/activate
   uv pip install -e ".[seer-dual-arm]"

此 extra 安装 ``seer-robokit-py==0.1.6``，与参考的 Seer 服务使用版本一致。RPent
自身的基础依赖仍然需要安装；此集成不需要 VLA、仿真器、相机或夹爪服务 extra。

预览
----

默认运行会读取回初始位配置并显示目标，不会导入或初始化 ``robokit``：

.. code-block:: bash

   uv run --extra seer-dual-arm rpent \
     --robot seer_dual_arm \
     --planner direct \
     --direct-tool return_to_initial \
     --service-root /home/seer/RBK/SeerTeleoperationService

默认情况下，``--service-root`` 会选择该目录下的 ``config/rbk_config.json``
和 ``config/controller/robokit.json``。可通过 ``--rbk-config`` 或
``--robokit-config`` 分别指定其他文件。机器人配置必须包含
``teleop.homing.enabled`` 和左右臂的 ``initial_poses``。

真机动作
--------

先检查预览并清空机械臂工作区。真机控制需要交互式终端，并显式添加 ``--live``：

.. code-block:: bash

   uv run --extra seer-dual-arm rpent \
     --robot seer_dual_arm \
     --planner direct \
     --direct-tool return_to_initial \
     --service-root /home/seer/RBK/SeerTeleoperationService \
     --live

控制器必须在 ``/RobotControl/WbcDebugTF`` 发布实测变换。动作会读取左右臂新鲜的末端反馈，
通过 smoothstep 位置插值和最短路径四元数插值向 ``/RobotControl/WbcEECommand`` 发布命令，
并等待位置和姿态误差连续满足配置阈值 ``stable_cycles`` 次。默认轨迹时长为 5 秒、超时为
30 秒、等待反馈 5 秒、命令频率 20 Hz；可通过 ``--duration``、``--timeout``、
``--feedback-timeout`` 和 ``--hz`` 调整。动作中断时，如果仍能读到新鲜反馈，会发布当前位置作为保持目标。

此第一步只控制两个机械臂末端，不会控制夹爪，也不会运行 VLA。正式操作前仍需在已连接的 Seer 控制器上验证实际动作。

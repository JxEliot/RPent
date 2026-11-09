Seer Dual-Arm Robot
===================

This integration provides one fixed action: return both end effectors to the
poses configured in ``teleop.homing.initial_poses`` in the Seer
``rbk_config.json``. It uses the robot's ``seer-robokit-py`` package and the
WBC command and measured-pose topics used by
``SeerTeleoperationService/return_to_initial.sh``. It does not start model,
camera, teleoperation, or gripper services. Both grippers are left unchanged.

Install
-------

Use a separate Python 3.12 environment for the controller SDK:

.. code-block:: bash

   uv venv --python 3.12
   source .venv/bin/activate
   uv pip install -e ".[seer-dual-arm]"

The extra adds ``seer-robokit-py==0.1.6``, matching the version used by the
referenced Seer service. RPent's base dependencies are still required; no VLA,
simulator, camera, or gripper-service extra is installed for this integration.

Preview
-------

The default run reads the homing configuration and prints the targets without
importing or initializing ``robokit``:

.. code-block:: bash

   uv run --extra seer-dual-arm rpent \
     --robot seer_dual_arm \
     --planner direct \
     --direct-tool return_to_initial \
     --service-root /home/seer/RBK/SeerTeleoperationService

By default, ``--service-root`` selects ``config/rbk_config.json`` and
``config/controller/robokit.json`` from that checkout. Use ``--rbk-config`` or
``--robokit-config`` to override either file. The target config must contain
``teleop.homing.enabled`` and both ``initial_poses``.

Live motion
-----------

Inspect the preview and clear the robot workspace before enabling live control.
The command requires an interactive terminal and the explicit ``--live`` flag:

.. code-block:: bash

   uv run --extra seer-dual-arm rpent \
     --robot seer_dual_arm \
     --planner direct \
     --direct-tool return_to_initial \
     --service-root /home/seer/RBK/SeerTeleoperationService \
     --live

The controller must publish measured transforms on
``/RobotControl/WbcDebugTF``. The action reads fresh measured left/right EE
poses, sends smoothstep position and shortest-path quaternion interpolation to
``/RobotControl/WbcEECommand``, and waits for configured position/orientation
tolerances to remain satisfied for ``stable_cycles``. Defaults are a 5-second
trajectory, 30-second timeout, 5-second feedback wait, and 20 Hz command rate;
override them with ``--duration``, ``--timeout``, ``--feedback-timeout``, and
``--hz``. If motion is interrupted while fresh feedback remains available, the
runner publishes a hold at the measured current pose.

This first step controls both arm end effectors only. It does not command either
gripper, and it does not run a VLA. Hardware behavior still needs to be
verified on the connected Seer controller before operational use.

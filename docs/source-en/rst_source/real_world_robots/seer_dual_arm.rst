Seer Dual-Arm Robot
===================

This integration provides one fixed action: return both end effectors to the
poses configured in ``teleop.homing.initial_poses`` in the Seer
``robots/seer_dual_arm/config/robot.json``. It uses the robot's ``seer-robokit-py`` package and the
WBC command and measured-pose topics used by
``SeerTeleoperationService/return_to_initial.sh``. The action is implemented
inside ``robots/seer_dual_arm/homing.py``; it calls the Python SDK directly,
without executing the shell script or importing the external service. The
configuration lives inside this integration. Both grippers are
left unchanged. No VLA, camera, or teleoperation service is needed.

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
The commands below use ``uv run --no-sync`` to run this environment without
resolving the repository's other optional robot dependencies.

Configuration
-------------

Edit files under ``robots/seer_dual_arm/config/``:

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - File
     - Purpose
   * - ``robot.json``
     - Left/right initial poses and arrival thresholds under ``teleop.homing``.
   * - ``robokit.json``
     - SDK DDS address, domain, and namespace for live mode.
   * - ``api.toml``
     - Model, endpoint, and credentials; loaded by default for the API planner when present.
   * - ``api.example.toml``
     - Credential-free API template committed to the repository.

The initial poses and DDS settings match the referenced local Seer service.
``pos`` is ``[x, y, z]`` in metres in ``base_link``; ``quat`` is ``[x, y, z, w]``.
The external checkout is not required with these defaults.

Create the local API file and fill in your relay settings:

.. code-block:: bash

   cp robots/seer_dual_arm/config/api.example.toml robots/seer_dual_arm/config/api.toml
   chmod 600 robots/seer_dual_arm/config/api.toml

Set ``model``, ``base_url``, and ``wire_api`` (``responses`` or ``chat``).
For bearer authentication, set ``api_key`` or name a credential environment
variable with ``env_key`` (default ``OPENAI_API_KEY``). Custom header authentication
uses ``[http_headers]``, for example ``x-bf-vk = "YOUR_RELAY_KEY"``.
``api.toml`` is ignored by Git and excluded from distribution packages.
Use ``--robot-config``, ``--robokit-config``, or ``--api-config`` to override a file.

Chat with GPT-6 Astra through a relay
-------------------------------------

Check the connection, then start a preview conversation:

.. code-block:: bash

   uv run --no-sync rpent-check-llm --planner api \
     --api-config robots/seer_dual_arm/config/api.toml

   uv run --no-sync rpent \
     --robot seer_dual_arm --planner api \
     --reasoning-effort low --tui

The terminal waits for your first message before contacting the model. Type
``回到初始位置`` to request the registered action. In preview mode the model
reports the targets and explains that the robot did not move. After each
response, enter another instruction or use ``/exit`` to close the conversation.
To execute real motion after checking the preview and clearing the workspace,
restart the same chat command with ``--live``. Starting the chat does not move
the robot; a user action request triggers the tool.

Use ``--model openai:gpt-6-astra`` to override a different configured model.
Responses uses ``openai:<model>``; Chat Completions uses ``openai-chat:<model>``.
You can alternatively select an existing Codex custom provider with
``--api-config ~/.codex/config.toml`` (alias ``--api-codex-config``).
This reads endpoint/model/header settings directly without starting Codex or
reading OAuth login data from ``auth.json``. The relay must support function
calls as well as text generation.

Chat with Claude
----------------

Configure ``ANTHROPIC_API_KEY`` (and ``ANTHROPIC_BASE_URL`` for a compatible
relay) as described in :doc:`../guides/configure_planner`, then run:

.. code-block:: bash

   uv run --no-sync rpent \
     --robot seer_dual_arm --planner claude_code \
     --model sonnet --tui

Enter ``回到初始位置`` at the blank ``you>`` prompt. Add ``--live`` at launch
for real motion. Claude receives only the homing and conversation-completion
tools for this integration; built-in shell and file tools are disabled by
default. Use ``/quit`` to exit. It uses the same internal homing implementation
as the API and direct planners.

Preview
-------

The default run reads the homing configuration and prints the targets without
importing or initializing ``robokit``:

.. code-block:: bash

   uv run --no-sync rpent \
     --robot seer_dual_arm \
     --planner direct \
     --direct-tool return_to_initial

The defaults use this integration's ``config/robot.json`` and ``config/robokit.json``.
An explicit ``--service-root`` selects ``config/rbk_config.json`` and
``config/controller/robokit.json`` from an external service checkout instead.
``--rbk-config`` remains an alias for ``--robot-config``. The target config must contain
both ``initial_poses``; if ``teleop.homing.enabled`` is present, it must be true.

Live motion
-----------

Inspect the preview and clear the robot workspace before enabling live control.
The command requires an interactive terminal and the explicit ``--live`` flag:

.. code-block:: bash

   uv run --no-sync rpent \
     --robot seer_dual_arm \
     --planner direct \
     --direct-tool return_to_initial \
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

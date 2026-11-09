# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from robots.seer_dual_arm.cartesian import execute_cartesian_delta
from robots.seer_dual_arm.homing import (
    _read_measured_poses,
    execute_return_to_initial,
    interpolate_poses,
    load_homing_config,
    pose_errors,
)


def write_config(path, *, stable_cycles=2):
    path.write_text(
        json.dumps(
            {
                "teleop": {
                    "homing": {
                        "enabled": True,
                        "position_threshold_m": 0.02,
                        "rotation_threshold_deg": 5.0,
                        "stable_cycles": stable_cycles,
                        "initial_poses": {
                            "left": {"pos": [0.63, 0.27, 1.05], "quat": [0, 0, 0, 1]},
                            "right": {
                                "pos": [0.63, -0.27, 1.05],
                                "quat": [0, 0, 0, -1],
                            },
                        },
                    }
                }
            }
        ),
        encoding="utf-8",
    )


def test_load_homing_config_normalizes_quaternions(tmp_path) -> None:
    path = tmp_path / "rbk_config.json"
    write_config(path)

    config = load_homing_config(path)

    assert config.target[:3] == (0.63, 0.27, 1.05)
    assert config.target[3:7] == (0.0, 0.0, 0.0, 1.0)
    assert config.target[10:14] == (0.0, 0.0, 0.0, -1.0)
    assert config.stable_cycles == 2


def test_load_homing_config_rejects_invalid_target(tmp_path) -> None:
    path = tmp_path / "rbk_config.json"
    write_config(path)
    raw = json.loads(path.read_text())
    raw["teleop"]["homing"]["initial_poses"]["left"]["quat"] = [0, 0, 0, 0]
    path.write_text(json.dumps(raw))

    with pytest.raises(ValueError, match="zero quaternion"):
        load_homing_config(path)


def test_load_homing_config_reports_non_numeric_pose_as_value_error(tmp_path) -> None:
    path = tmp_path / "rbk_config.json"
    write_config(path)
    raw = json.loads(path.read_text())
    raw["teleop"]["homing"]["initial_poses"]["left"]["pos"][0] = None
    path.write_text(json.dumps(raw))

    with pytest.raises(
        ValueError, match="left initial pose pos must contain finite values"
    ):
        load_homing_config(path)


def test_interpolation_and_pose_errors_handle_quaternion_sign() -> None:
    start = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0) * 2
    target = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, -1.0) * 2

    midpoint = interpolate_poses(start, target, 0.5)
    signed_target = (
        target[:3]
        + tuple(-value for value in target[3:7])
        + target[7:10]
        + tuple(-value for value in target[10:14])
    )
    errors = pose_errors(target, signed_target)

    assert midpoint[:3] == pytest.approx((0.5, 1.0, 1.5))
    assert midpoint[3:7] == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert errors[0] == pytest.approx((0.0, 0.0))
    assert errors[1] == pytest.approx((0.0, 0.0))


def test_measured_feedback_resolves_base_to_both_ee_frames() -> None:
    def transform(parent, child, xyz):
        return SimpleNamespace(
            frameid=parent,
            childframeid=child,
            translation=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )

    message = SimpleNamespace(
        transform=[
            transform("base_link", "left_ee_link", (0.6, 0.2, 1.0)),
            transform("base_link", "right_ee_link", (0.6, -0.2, 1.0)),
        ]
    )

    assert _read_measured_poses(message) == pytest.approx(
        (0.6, 0.2, 1.0, 0, 0, 0, 1, 0.6, -0.2, 1.0, 0, 0, 0, 1)
    )


def test_preview_does_not_initialize_robot_sdk(tmp_path) -> None:
    config_path = tmp_path / "rbk_config.json"
    write_config(config_path)

    result = execute_return_to_initial(
        rbk_config=config_path,
        robokit_config=tmp_path / "unused.json",
        live=False,
        duration=1.0,
        timeout=2.0,
        feedback_timeout=1.0,
        hz=20.0,
        node_name="test",
        check_cancelled=lambda: None,
    )

    assert result["success"] is True
    assert result["moved"] is False
    assert result["grippers_changed"] is False
    assert result["target_xyzw"][:3] == [0.63, 0.27, 1.05]


@pytest.fixture
def homing_sdk(tmp_path: Path, monkeypatch):
    config_path = tmp_path / "rbk_config.json"
    write_config(config_path, stable_cycles=2)
    robokit_config = tmp_path / "robokit.json"
    robokit_config.write_text("{}", encoding="utf-8")

    class Header:
        seq = 0
        dataNsec = 0
        pubNsec = 0
        frameID = ""

    class Feedback:
        def __init__(self, sequence, measured):
            self.header = SimpleNamespace(
                seq=sequence, dataNsec=sequence, pubNsec=sequence
            )
            self.transform = [
                feedback_transform("left_ee_link", measured[:7]),
                feedback_transform("right_ee_link", measured[7:]),
            ]

        def SerializeToString(self):
            return str(self.header.seq).encode()

    class Command:
        def __init__(self):
            self.header = Header()
            self.leftEEPose = SimpleNamespace()
            self.rightEEPose = SimpleNamespace()
            self.leftEEVel = SimpleNamespace()
            self.rightEEVel = SimpleNamespace()

    class DataPool:
        def __init__(self):
            self.sequence = 0
            self.published = []
            self.feedback_available = True
            self.feedback_updates = True
            self.follow_commands = False
            self.measured = (0.63, 0.27, 1.05, 0, 0, 0, 1) + (
                0.63,
                -0.27,
                1.05,
                0,
                0,
                0,
                1,
            )
            self.reader = SimpleNamespace(is_shared_latest=True, closed=False)

        def subscribe(self, *args):
            self.subscribed = args
            return self.reader

        def unsubscribe(self, reader):
            raise ValueError("shared latest reader cannot be unsubscribed")

        def publish(self, *args):
            self.publisher = args

        def get(self, *args):
            if not self.feedback_available:
                return None
            if self.feedback_updates:
                self.sequence += 1
            return Feedback(self.sequence, self.measured)

        def put(self, topic, message):
            self.published.append((topic, message))
            if self.follow_commands:
                self.measured = tuple(
                    getattr(pose, field)
                    for pose in (message.leftEEPose, message.rightEEPose)
                    for field in ("x", "y", "z", "qx", "qy", "qz", "qw")
                )

    def feedback_transform(child, pose):
        return SimpleNamespace(
            frameid="base_link",
            childframeid=child,
            translation=SimpleNamespace(x=pose[0], y=pose[1], z=pose[2]),
            rotation=SimpleNamespace(x=pose[3], y=pose[4], z=pose[5], w=pose[6]),
        )

    data_pool = DataPool()
    system_calls = []

    class System:
        shut_down = False

        def init(self, *args, **kwargs):
            if self.shut_down:
                raise RuntimeError("Robokit has already shut down")
            system_calls.append(("init", args, kwargs))

        def shutdown(self):
            self.shut_down = True
            data_pool.reader.closed = True
            system_calls.append(("shutdown",))

    system = System()
    robokit = ModuleType("robokit")
    core = ModuleType("robokit.core")
    core.datapool = data_pool
    core.system = system
    protocol = ModuleType("robokit.protocol")
    protocol.__path__ = []
    feedback_module = ModuleType("robokit.protocol.messageV4_stampedtransform_pb2")
    feedback_module.MessageV4_StampedTransforms = Feedback
    command_module = ModuleType("robokit.protocol.messageV4_wbccmd_pb2")
    command_module.WBCCommand = Command
    for name, module in (
        ("robokit", robokit),
        ("robokit.core", core),
        ("robokit.protocol", protocol),
        (
            "robokit.protocol.messageV4_stampedtransform_pb2",
            feedback_module,
        ),
        ("robokit.protocol.messageV4_wbccmd_pb2", command_module),
    ):
        monkeypatch.setitem(sys.modules, name, module)

    return SimpleNamespace(
        pool=data_pool,
        system_calls=system_calls,
        command_type=Command,
        runtime={
            "rbk_config": config_path,
            "robokit_config": robokit_config,
            "live": True,
            "duration": 0.01,
            "timeout": 1.0,
            "feedback_timeout": 0.1,
            "hz": 1000.0,
            "node_name": "test",
        },
    )


def test_live_run_publishes_dual_arm_wbc_and_waits_for_stable_feedback(
    homing_sdk,
) -> None:
    result = execute_return_to_initial(
        **homing_sdk.runtime, check_cancelled=lambda: None
    )
    data_pool = homing_sdk.pool
    assert result["success"] is True
    assert result["moved"] is True
    assert data_pool.publisher == (
        "/RobotControl/WbcEECommand",
        homing_sdk.command_type,
    )
    assert data_pool.published
    topic, command = data_pool.published[-1]
    assert topic == "/RobotControl/WbcEECommand"
    assert command.header.frameID == "base_link"
    assert command.leftEEPose.x == pytest.approx(0.63)
    assert command.rightEEPose.y == pytest.approx(-0.27)
    assert command.leftEEVel.linear_x == 0.0
    assert homing_sdk.system_calls[-1] == ("shutdown",)
    assert data_pool.reader.closed


@pytest.mark.parametrize("feedback", ["missing", "cached"])
def test_missing_or_cached_feedback_preserves_error_and_closes_sdk(
    homing_sdk, feedback: str
) -> None:
    homing_sdk.pool.feedback_available = feedback != "missing"
    homing_sdk.pool.feedback_updates = feedback != "cached"
    with pytest.raises(RuntimeError, match="no fresh measured EE feedback"):
        execute_return_to_initial(**homing_sdk.runtime, check_cancelled=lambda: None)
    assert not homing_sdk.pool.published
    assert homing_sdk.system_calls[-1] == ("shutdown",)
    assert homing_sdk.pool.reader.closed


def test_toolkit_can_home_again_after_feedback_failure_and_success(
    tmp_path: Path, homing_sdk
) -> None:
    from robots.seer_dual_arm.toolkit import SeerDualArmToolkit
    from rpent.dashboard.events import NullDashboardEventSink
    from rpent.memory import MemoryManager

    toolkit = SeerDualArmToolkit(
        runtime_kwargs=homing_sdk.runtime,
        dashboard_events=NullDashboardEventSink(),
        memory=MemoryManager(tmp_path / "memory"),
        state_output_dir=tmp_path / "states",
        interactive=True,
    )
    try:
        homing_sdk.pool.feedback_available = False
        failed = toolkit.execute_tool("return_to_initial", {})
        assert failed.data["success"] is False
        assert "no fresh measured EE feedback" in failed.data["error"]
        homing_sdk.pool.feedback_available = True
        for _ in range(2):
            result = toolkit.execute_tool("return_to_initial", {})
            assert result.data["success"] is True
            assert result.data["moved"] is True
            assert not homing_sdk.pool.reader.closed
        assert [call[0] for call in homing_sdk.system_calls] == ["init"]
    finally:
        toolkit.close()
    assert homing_sdk.pool.reader.closed
    assert [call[0] for call in homing_sdk.system_calls] == ["init", "shutdown"]
    toolkit.close()
    assert [call[0] for call in homing_sdk.system_calls] == ["init", "shutdown"]


@pytest.mark.parametrize("action", ["move_delta", "rotate_delta"])
def test_delta_preview_does_not_import_sdk_or_invent_measured_target(
    tmp_path, monkeypatch, action
) -> None:
    path = tmp_path / "robot.json"
    write_config(path)
    monkeypatch.setitem(sys.modules, "robokit", None)
    result = execute_cartesian_delta(
        arm=" LEFT ",
        delta=[0, 0, 0.02],
        action=action,
        rbk_config=path,
        robokit_config=tmp_path / "unused.json",
        live=False,
        duration=1,
        timeout=2,
        feedback_timeout=1,
        hz=20,
        node_name="test",
        check_cancelled=lambda: None,
    )
    assert result["success"] is True
    assert result["moved"] is False
    assert result["target_frame"] == "base_link"
    assert result["arm"] == "left"
    assert result["held_arm"] == "right"
    assert result["grippers_changed"] is False
    assert not {"target_xyzw", "start_xyzw", "measured_xyzw"} & result.keys()


@pytest.mark.parametrize(
    ("action", "arm", "delta"),
    [
        ("move_delta", "unknown", [0, 0, 0.01]),
        ("rotate_delta", "both", [0, 0, 0.01]),
        ("move_delta", "left", [0, 0]),
        ("move_delta", "left", [False, 0, 0]),
        ("move_delta", "left", [float("nan"), 0, 0]),
        ("rotate_delta", "right", [0, float("inf"), 0]),
        ("move_delta", "left", [0.08, 0.08, 0]),
        ("rotate_delta", "right", [0.4, 0.4, 0]),
    ],
)
def test_invalid_delta_is_rejected_before_sdk_init(homing_sdk, action, arm, delta):
    with pytest.raises(ValueError):
        execute_cartesian_delta(
            action=action,
            arm=arm,
            delta=delta,
            **homing_sdk.runtime,
            check_cancelled=lambda: None,
        )
    assert not homing_sdk.system_calls
    assert not homing_sdk.pool.published


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("max_translation_m", 0),
        ("max_rotation_rad", float("nan")),
        ("stable_cycles", True),
        ("position_threshold_m", -1),
    ],
)
def test_invalid_cartesian_config_is_rejected_before_motion(homing_sdk, key, value):
    path = homing_sdk.runtime["rbk_config"]
    raw = json.loads(path.read_text())
    raw["teleop"]["cartesian"] = {key: value}
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError):
        execute_cartesian_delta(
            action="move_delta",
            arm="left",
            delta=[0, 0, 0.01],
            **homing_sdk.runtime,
            check_cancelled=lambda: None,
        )
    assert not homing_sdk.system_calls
    assert not homing_sdk.pool.published


@pytest.mark.parametrize("arm", ["left", "right"])
def test_translation_uses_measured_start_and_holds_orientation_and_other_arm(
    homing_sdk, arm
):
    pool = homing_sdk.pool
    pool.follow_commands = True
    start = pool.measured
    delta = [0.01, -0.02, 0.03]
    result = execute_cartesian_delta(
        action="move_delta",
        arm=arm,
        delta=delta,
        **homing_sdk.runtime,
        check_cancelled=lambda: None,
    )
    offset = 0 if arm == "left" else 7
    other = 7 - offset
    target = list(start)
    target[offset : offset + 3] = [start[offset + i] + delta[i] for i in range(3)]
    assert result["success"] and result["moved"]
    assert result["start_xyzw"] == list(start)
    assert result["target_xyzw"] == pytest.approx(target)
    assert result["measured_xyzw"] == pytest.approx(target)
    for _, command in pool.published:
        pose = getattr(command, f"{arm}EEPose")
        assert [pose.qx, pose.qy, pose.qz, pose.qw] == pytest.approx(
            start[offset + 3 : offset + 7]
        )
        held = getattr(command, f"{'right' if arm == 'left' else 'left'}EEPose")
        assert [
            getattr(held, field) for field in ("x", "y", "z", "qx", "qy", "qz", "qw")
        ] == pytest.approx(start[other : other + 7])
        assert command.header.frameID == "base_link"
    assert len(pool.published) > 3  # Interpolation plus measured arrival, not one jump.


def test_rotation_is_extrinsic_xyz_in_base_frame_and_preserves_positions(homing_sdk):
    pool = homing_sdk.pool
    pool.follow_commands = True
    # Non-identity start makes base-frame pre-multiplication observable.
    s = math.sqrt(0.5)
    pool.measured = (*pool.measured[:10], s, 0, 0, s)
    start = pool.measured
    roll, pitch, yaw = 0.1, -0.2, 0.3
    result = execute_cartesian_delta(
        action="rotate_delta",
        arm="right",
        delta=[roll, pitch, yaw],
        **homing_sdk.runtime,
        check_cancelled=lambda: None,
    )
    # Independent Euler matrix: Rz(yaw) Ry(pitch) Rx(roll + pi/2).
    cr, sr = math.cos(roll + math.pi / 2), math.sin(roll + math.pi / 2)
    cp, sp, cy, sy = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    expected = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    x, y, z, w = result["target_xyzw"][10:14]
    actual = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    for actual_row, expected_row in zip(actual, expected, strict=True):
        assert actual_row == pytest.approx(expected_row)
    for _, command in pool.published:
        assert [
            command.rightEEPose.x,
            command.rightEEPose.y,
            command.rightEEPose.z,
        ] == pytest.approx(start[7:10])
        assert command.leftEEPose.x == pytest.approx(start[0])
    assert result["measured_xyzw"] == pytest.approx(result["target_xyzw"])


@pytest.mark.parametrize("feedback", ["missing", "cached"])
def test_delta_does_not_command_without_fresh_start(homing_sdk, feedback):
    homing_sdk.pool.feedback_available = feedback != "missing"
    homing_sdk.pool.feedback_updates = feedback != "cached"
    with pytest.raises(RuntimeError, match="no fresh measured EE feedback"):
        execute_cartesian_delta(
            action="move_delta",
            arm="left",
            delta=[0, 0, 0.02],
            **homing_sdk.runtime,
            check_cancelled=lambda: None,
        )
    assert not homing_sdk.pool.published


def test_cancelled_delta_sends_measured_hold_and_retains_supplied_session(homing_sdk):
    from robots.seer_dual_arm.homing import HomingSession

    pool = homing_sdk.pool
    pool.follow_commands = True
    session = HomingSession(
        node_name="test", config_path=homing_sdk.runtime["robokit_config"]
    )

    def cancel_after_first_command():
        if pool.published:
            raise RuntimeError("operator cancelled")

    try:
        with pytest.raises(RuntimeError, match="operator cancelled"):
            execute_cartesian_delta(
                action="move_delta",
                arm="left",
                delta=[0, 0, 0.02],
                **homing_sdk.runtime,
                check_cancelled=cancel_after_first_command,
                session=session,
            )
        assert len(pool.published) == 2
        hold = pool.published[-1][1]
        assert hold.leftEEPose.z == pytest.approx(pool.measured[2])
        assert not pool.reader.closed
    finally:
        session.close()


def test_toolkit_shares_one_sdk_session_for_homing_translation_and_rotation(
    tmp_path, homing_sdk
):
    from robots.seer_dual_arm.toolkit import SeerDualArmToolkit
    from rpent.dashboard.events import NullDashboardEventSink
    from rpent.memory import MemoryManager

    toolkit = SeerDualArmToolkit(
        runtime_kwargs=homing_sdk.runtime,
        dashboard_events=NullDashboardEventSink(),
        memory=MemoryManager(tmp_path / "memory"),
        state_output_dir=tmp_path / "states",
        interactive=True,
    )
    homing_sdk.pool.follow_commands = True
    try:
        for name, args in [
            ("return_to_initial", {}),
            ("move_delta", {"arm": " LEFT ", "delta_xyz": [0, 0, 0.02]}),
            ("rotate_delta", {"arm": "right", "delta_rpy": [0, 0, 0.1]}),
        ]:
            result = toolkit.execute_tool(name, args)
            assert result.data["success"], result.data
            assert result.data["moved"]
            assert not homing_sdk.pool.reader.closed
        assert [call[0] for call in homing_sdk.system_calls] == ["init"]
    finally:
        toolkit.close()
    assert [call[0] for call in homing_sdk.system_calls] == ["init", "shutdown"]
    toolkit.close()
    assert [call[0] for call in homing_sdk.system_calls] == ["init", "shutdown"]


def test_delta_times_out_when_feedback_never_reaches_target(homing_sdk):
    homing_sdk.runtime["timeout"] = 0.05
    start = homing_sdk.pool.measured
    with pytest.raises(RuntimeError, match="move_delta timed out"):
        execute_cartesian_delta(
            action="move_delta",
            arm="left",
            delta=[0, 0, 0.02],
            **homing_sdk.runtime,
            check_cancelled=lambda: None,
        )
    hold = homing_sdk.pool.published[-1][1]
    assert hold.leftEEPose.z == pytest.approx(start[2])
    assert homing_sdk.pool.reader.closed


def test_cached_arrival_sample_does_not_count_as_stable_feedback(
    homing_sdk, monkeypatch
):
    pool = homing_sdk.pool
    pool.follow_commands = True
    original_put = pool.put

    def freeze_at_target(topic, command):
        original_put(topic, command)
        if command.leftEEPose.z == pytest.approx(1.07):
            pool.feedback_updates = False

    monkeypatch.setattr(pool, "put", freeze_at_target)
    with pytest.raises(RuntimeError, match="EE feedback stale"):
        execute_cartesian_delta(
            action="move_delta",
            arm="left",
            delta=[0, 0, 0.02],
            **homing_sdk.runtime,
            check_cancelled=lambda: None,
        )
    assert pool.reader.closed


def test_nonfinite_measured_pose_is_rejected_without_commanding(homing_sdk):
    homing_sdk.pool.measured = (float("nan"), *homing_sdk.pool.measured[1:])
    with pytest.raises(ValueError, match="measured EE pose must contain finite values"):
        execute_cartesian_delta(
            action="move_delta",
            arm="left",
            delta=[0, 0, 0.02],
            **homing_sdk.runtime,
            check_cancelled=lambda: None,
        )
    assert not homing_sdk.pool.published


def test_both_arms_translate_together_in_each_wbc_command(homing_sdk):
    pool = homing_sdk.pool
    pool.follow_commands = True
    start = pool.measured
    result = execute_cartesian_delta(
        action="move_delta",
        arm="both",
        delta=[0, 0, 0.10],
        **homing_sdk.runtime,
        check_cancelled=lambda: None,
    )
    assert result["success"] and result["moved"]
    assert result["held_arm"] is None
    assert result["measured_xyzw"][2] == pytest.approx(start[2] + 0.1)
    assert result["measured_xyzw"][9] == pytest.approx(start[9] + 0.1)
    for _, command in pool.published:
        assert command.leftEEPose.z - start[2] == pytest.approx(
            command.rightEEPose.z - start[9]
        )
        for offset, pose in [(0, command.leftEEPose), (7, command.rightEEPose)]:
            assert [pose.x, pose.y] == pytest.approx(start[offset : offset + 2])
            assert [pose.qx, pose.qy, pose.qz, pose.qw] == pytest.approx(
                start[offset + 3 : offset + 7]
            )


def test_both_arm_move_requires_both_measured_arrivals(homing_sdk, monkeypatch):
    pool = homing_sdk.pool
    homing_sdk.runtime["timeout"] = 0.05
    original_put = pool.put

    def only_left_follows(topic, command):
        original_put(topic, command)
        left = command.leftEEPose
        pool.measured = (
            left.x,
            left.y,
            left.z,
            left.qx,
            left.qy,
            left.qz,
            left.qw,
            *pool.measured[7:],
        )

    monkeypatch.setattr(pool, "put", only_left_follows)
    with pytest.raises(RuntimeError, match="move_delta timed out"):
        execute_cartesian_delta(
            action="move_delta",
            arm="both",
            delta=[0, 0, 0.10],
            **homing_sdk.runtime,
            check_cancelled=lambda: None,
        )

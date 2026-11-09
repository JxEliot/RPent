# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

from __future__ import annotations

import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

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


def test_live_run_publishes_dual_arm_wbc_and_waits_for_stable_feedback(
    tmp_path: Path, monkeypatch
) -> None:
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
        def __init__(self, sequence):
            self.header = SimpleNamespace(
                seq=sequence, dataNsec=sequence, pubNsec=sequence
            )
            self.transform = [
                feedback_transform("left_ee_link", (0.63, 0.27, 1.05)),
                feedback_transform("right_ee_link", (0.63, -0.27, 1.05)),
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

        def subscribe(self, *args):
            self.subscribed = args

        def publish(self, *args):
            self.publisher = args

        def get(self, *args):
            self.sequence += 1
            return Feedback(self.sequence)

        def put(self, topic, message):
            self.published.append((topic, message))

    def feedback_transform(child, xyz):
        return SimpleNamespace(
            frameid="base_link",
            childframeid=child,
            translation=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )

    data_pool = DataPool()
    system_calls = []
    system = SimpleNamespace(
        init=lambda *args, **kwargs: system_calls.append(("init", args, kwargs)),
        shutdown=lambda: system_calls.append(("shutdown",)),
    )
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

    result = execute_return_to_initial(
        rbk_config=config_path,
        robokit_config=robokit_config,
        live=True,
        duration=0.01,
        timeout=1.0,
        feedback_timeout=0.1,
        hz=1000.0,
        node_name="test",
        check_cancelled=lambda: None,
    )

    assert result["success"] is True
    assert result["moved"] is True
    assert data_pool.publisher == ("/RobotControl/WbcEECommand", Command)
    assert data_pool.published
    topic, command = data_pool.published[-1]
    assert topic == "/RobotControl/WbcEECommand"
    assert command.header.frameID == "base_link"
    assert command.leftEEPose.x == pytest.approx(0.63)
    assert command.rightEEPose.y == pytest.approx(-0.27)
    assert command.leftEEVel.linear_x == 0.0
    assert system_calls[-1] == ("shutdown",)

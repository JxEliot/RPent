# Copyright 2026 The RPent Authors.
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

import json
import sys
import time
from types import ModuleType, SimpleNamespace

import pytest

from robots.seer_dual_arm.gripper import execute_gripper


@pytest.fixture
def config(tmp_path):
    path = tmp_path / "robot.json"
    path.write_text(
        json.dumps(
            {
                "teleop": {
                    "gripper": {
                        "status_timeout_s": 0.03,
                    }
                }
            }
        )
    )
    return path


class FakeSession:
    def start(self):
        pass


class FakeTransport:
    def __init__(self):
        self.calls = []
        self.sequence = 0
        self.sent = {}
        self.heartbeat = True
        self.new_feedback = True
        self.fail_side = None
        self.error = {}

    def subscribe(self, topic, message_type):
        assert topic == "/GripperService/Status"

    def publish(self, topic, message_type):
        pass

    def put(self, topic, message):
        side = "left" if topic.endswith("000") else "right"
        self.calls.append((topic, message))
        if side == self.fail_side:
            raise OSError("transport failed")
        self.sent[side] = message.GripFinger.fingerinfos.control

    def get(self, topic, message_type):
        if self.heartbeat or not self.sequence:
            self.sequence += 1
        now = time.time_ns()
        feedback, applied = {}, {}
        for side in ("left", "right"):
            command = self.sent.get(side)
            feedback[side] = {
                "feedback_fresh": True,
                "feedback_sequence": self.sequence if self.new_feedback else 1,
                "timestamp_ns": now,
                "fault_code": 0,
                "state_code": 0,
                "measured_position": command.Flexion if command else 50,
            }
            if command:
                applied[side] = {
                    "position": command.Flexion,
                    "velocity": command.vel,
                    "acceleration": command.Acc,
                    "deceleration": command.Dec,
                    "force": command.force,
                }
        status = {
            "schema": "seer.gripper.status.v1",
            "sequence": self.sequence,
            "emitted_unix_ns": now if self.heartbeat else self.first_timestamp,
            "feedback": feedback,
            "applied": applied,
            "errors": self.error,
        }
        return SimpleNamespace(
            data=[
                SimpleNamespace(key="gripper_status", string_value=json.dumps(status))
            ]
        )

    @property
    def first_timestamp(self):
        if not hasattr(self, "_first_timestamp"):
            self._first_timestamp = time.time_ns()
        return self._first_timestamp


@pytest.fixture
def transport(monkeypatch):
    transport = FakeTransport()
    core = ModuleType("robokit.core")
    core.datapool = transport
    grip = ModuleType("robokit.protocol.messageV4_grip_pb2")
    grip.THUMB = 1

    def grip_message():
        control = SimpleNamespace()
        return SimpleNamespace(
            GripFinger=SimpleNamespace(
                type=0, fingerinfos=SimpleNamespace(control=control)
            )
        )

    grip.MessageV4_grip = grip_message
    universal = ModuleType("robokit.protocol.messageV4_universal_pb2")
    universal.MessageV4_UniversalDatas = type("Universal", (), {})
    for name, module in {
        "robokit": ModuleType("robokit"),
        "robokit.core": core,
        "robokit.protocol": ModuleType("robokit.protocol"),
        "robokit.protocol.messageV4_grip_pb2": grip,
        "robokit.protocol.messageV4_universal_pb2": universal,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    return transport


def run(config, *, action="open", arm="both", live=True, position=None):
    return execute_gripper(
        action=action,
        arm=arm,
        position=position,
        rbk_config=config,
        robokit_config="unused",
        live=live,
        node_name="test",
        check_cancelled=lambda: None,
        session=FakeSession(),
    )


@pytest.mark.parametrize("action", ["open", "close", "set", "state"])
def test_preview_never_connects(config, action):
    result = run(config, action=action, live=False, position=50)
    assert result["success"] and not result["command_published_sides"]
    assert not result["command_attempted_sides"]


@pytest.mark.parametrize("position", [-1, 101, float("nan"), True])
def test_invalid_position_rejected_before_transport(config, position):
    with pytest.raises(ValueError):
        run(config, action="set", position=position)


@pytest.mark.parametrize(
    ("action", "arm", "target", "topics"),
    [
        ("open", "left", 100, ["/GripCmd/Gripper-000"]),
        ("close", "right", 0, ["/GripCmd/Gripper-001"]),
        ("set", "both", 37, ["/GripCmd/Gripper-000", "/GripCmd/Gripper-001"]),
    ],
)
def test_commands_without_status(
    config, transport, action, arm, target, topics, monkeypatch
):
    def unexpected_status(*args):
        raise AssertionError("gripper commands must not read status")

    monkeypatch.setattr(transport, "subscribe", unexpected_status)
    monkeypatch.setattr(transport, "get", unexpected_status)
    result = run(config, action=action, arm=arm, position=target)
    assert result["success"] and "not checked" in result["summary"]
    assert result["command_published_sides"] == (
        ["left", "right"] if arm == "both" else [arm]
    )
    assert [topic for topic, _ in transport.calls] == topics
    for _, message in transport.calls:
        assert message.GripFinger.type == 1
        control = message.GripFinger.fingerinfos.control
        assert (
            control.Flexion,
            control.vel,
            control.Acc,
            control.Dec,
            control.force,
        ) == (target, 100, 100, 100, 100)


def test_state_does_not_send_a_command(config, transport):
    result = run(config, action="state")
    assert result["success"] and result["status"]["sequence"] >= 2
    assert not transport.calls


def test_stopped_service_prevents_state_read(config, transport):
    transport.heartbeat = False
    result = run(config, action="state")
    assert not result["success"] and "status" in result["error"]
    assert not transport.calls


def test_partial_send_reports_uncertainty(config, transport):
    transport.fail_side = "right"
    result = run(config)
    assert not result["success"] and result["grippers_changed"]
    assert result["command_attempted_sides"] == ["left", "right"]
    assert result["command_published_sides"] == ["left"]
    assert "transport failed" in result["error"]

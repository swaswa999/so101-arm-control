# Modified for SO101 Arm Control; see the repository THIRD_PARTY.md.
# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Tests for lelab.teleoperate — request schema and status handlers."""

from __future__ import annotations

import pytest


def test_teleoperate_request_rejects_missing_fields() -> None:
    from pydantic import ValidationError

    from lelab.teleoperate import TeleoperateRequest

    with pytest.raises(ValidationError):
        TeleoperateRequest()


def test_handle_teleoperation_status_returns_dict() -> None:
    from lelab.teleoperate import handle_teleoperation_status

    result = handle_teleoperation_status()
    assert isinstance(result, dict)


def test_handle_get_joint_positions_returns_dict_when_idle() -> None:
    from lelab.teleoperate import handle_get_joint_positions

    result = handle_get_joint_positions()
    assert isinstance(result, dict)


def test_get_joint_positions_from_robot_uses_provided_object() -> None:
    from lelab.teleoperate import get_joint_positions_from_robot
    from tests.mocks import FakeRobot

    robot = FakeRobot()
    robot.connect()
    positions = get_joint_positions_from_robot(robot)
    assert isinstance(positions, dict)


def test_get_joint_positions_maps_degrees_to_radians_without_correction() -> None:
    """Every joint maps its Present_Position (degrees around the calibration
    center) straight to radians, deg * pi/180, with no per-joint offset.
    shoulder_lift/elbow_flex used to be run through a correction table; a
    calibration that would have triggered it is supplied here to prove the
    offset is gone and they now map like every other joint."""
    import math

    from lelab.teleoperate import get_joint_positions_from_robot

    class _Cal:
        # The removed correction derived its offset from range_min/range_max.
        range_min = 0
        range_max = 4095

    class _Robot:
        calibration = {"shoulder_lift": _Cal(), "elbow_flex": _Cal()}

        def get_observation(self):
            return {
                "shoulder_pan.pos": 0.0,
                "shoulder_lift.pos": 90.0,
                "elbow_flex.pos": -45.0,
                "wrist_flex.pos": 30.0,
                "wrist_roll.pos": 12.0,
                "gripper.pos": 50.0,
            }

    positions = get_joint_positions_from_robot(_Robot())

    assert positions["Rotation"] == pytest.approx(0.0)
    assert positions["Pitch"] == pytest.approx(math.radians(90.0))
    assert positions["Elbow"] == pytest.approx(math.radians(-45.0))
    assert positions["Wrist_Pitch"] == pytest.approx(math.radians(30.0))
    assert positions["Wrist_Roll"] == pytest.approx(math.radians(12.0))
    assert positions["Jaw"] == pytest.approx(math.radians(50.0))


def test_start_teleoperation_reports_connection_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A device that fails to connect must make the start handler return
    success=False (so the UI surfaces the error and doesn't navigate to an
    empty teleop screen) and reset state so a retry isn't blocked. Previously
    the connect ran in a worker thread and the handler always claimed success.
    """
    import lelab.teleoperate as teleop

    monkeypatch.setattr(teleop, "teleoperation_active", False)
    monkeypatch.setattr(teleop, "setup_calibration_files", lambda leader, follower: ("leader", "follower"))

    class _Bus:
        def connect(self) -> None:
            raise RuntimeError("serial port unavailable")

    class _Device:
        def __init__(self, config) -> None:
            self.bus = _Bus()
            self.cameras: dict = {}
            self.disconnected = False

        def disconnect(self) -> None:
            self.disconnected = True

    monkeypatch.setattr(teleop, "SO101Follower", _Device)
    monkeypatch.setattr(teleop, "SO101Leader", _Device)

    request = teleop.TeleoperateRequest(
        leader_port="COM_LEADER",
        follower_port="COM_FOLLOWER",
        leader_config="leader",
        follower_config="follower",
    )
    result = teleop.handle_start_teleoperation(request)

    assert result["success"] is False
    # The message must name the arm that failed (the follower connects first).
    assert "follower" in result["message"].lower()
    assert "COM_FOLLOWER" in result["message"]
    # State must be reset so the next attempt isn't blocked by the mutex.
    assert teleop.teleoperation_active is False


def test_start_teleoperation_disconnects_follower_when_leader_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The partial-connect path: if the follower connects but the leader then
    fails, the follower must be disconnected so its serial port is released.
    """
    import lelab.teleoperate as teleop

    monkeypatch.setattr(teleop, "teleoperation_active", False)
    monkeypatch.setattr(teleop, "setup_calibration_files", lambda leader, follower: ("leader", "follower"))

    class _OkBus:
        def connect(self) -> None:
            pass

    class _FailingBus:
        def connect(self) -> None:
            raise RuntimeError("leader offline")

    class _Follower:
        def __init__(self, config) -> None:
            self.bus = _OkBus()
            self.cameras: dict = {}
            self.disconnected = False

        def disconnect(self) -> None:
            self.disconnected = True

    class _Leader:
        def __init__(self, config) -> None:
            self.bus = _FailingBus()
            self.disconnected = False

        def disconnect(self) -> None:
            self.disconnected = True

    created: dict = {}
    monkeypatch.setattr(
        teleop, "SO101Follower", lambda config: created.setdefault("follower", _Follower(config))
    )
    monkeypatch.setattr(teleop, "SO101Leader", lambda config: created.setdefault("leader", _Leader(config)))

    request = teleop.TeleoperateRequest(
        leader_port="COM_LEADER",
        follower_port="COM_FOLLOWER",
        leader_config="leader",
        follower_config="follower",
    )
    result = teleop.handle_start_teleoperation(request)

    assert result["success"] is False
    assert "leader" in result["message"].lower()
    # The already-connected follower must have been cleaned up.
    assert created["follower"].disconnected is True
    assert teleop.teleoperation_active is False


def test_joint_positions_endpoint_does_not_touch_serial_bus(monkeypatch):
    import lelab.teleoperate as teleop

    class Robot:
        def get_observation(self):
            raise AssertionError("HTTP must not race the serial worker")

    monkeypatch.setattr(teleop, "teleoperation_active", True)
    monkeypatch.setattr(teleop, "current_robot", Robot())
    monkeypatch.setattr(teleop, "_last_joint_positions", {"Rotation": 0.5})
    result = teleop.handle_get_joint_positions()
    assert result["success"] is True
    assert result["joint_positions"] == {"Rotation": 0.5}


def test_exhausted_position_reads_stop_worker_and_release_both_arms(monkeypatch):
    import lelab.teleoperate as teleop

    monkeypatch.setattr(teleop, "teleoperation_active", False)
    monkeypatch.setattr(teleop, "setup_calibration_files", lambda *args: ("leader", "follower"))
    created = []

    class Bus:
        def connect(self):
            pass

        def write_calibration(self, calibration):
            pass

        def sync_read(self, *args, **kwargs):
            raise ConnectionError("There is no status packet!")

    class Device:
        def __init__(self, config):
            self.bus = Bus()
            self.cameras = {}
            self.calibration = {}
            self.disconnected = False
            created.append(self)

        def configure(self):
            pass

        def disconnect(self):
            self.disconnected = True

        def send_action(self, action):
            raise AssertionError("Never send motion after an unsuccessful leader read")

    class SynchronousThread:
        def __init__(self, target, **kwargs):
            self.target = target

        def start(self):
            self.target()

    monkeypatch.setattr(teleop, "SO101Follower", Device)
    monkeypatch.setattr(teleop, "SO101Leader", Device)
    monkeypatch.setattr(teleop.threading, "Thread", SynchronousThread)
    teleop.handle_start_teleoperation(teleop.TeleoperateRequest(
        leader_port="leader", follower_port="follower",
        leader_config="leader.json", follower_config="follower.json"))
    assert teleop.teleoperation_active is False
    assert all(device.disconnected for device in created)
    assert "Leader position read failed" in teleop.handle_teleoperation_status()["last_error"]

"""Lifecycle tests for the single-shot grid generator node."""

from __future__ import annotations

import importlib
import sys
import types


def test_grid_generator_does_not_run_from_constructor(monkeypatch, tmp_path) -> None:
    """Construction sets up ROS only; generation is triggered by run()."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))

    class FakeLogger:
        def warning(self, message: str) -> None:
            self.last_warning = message

    class FakeNode:
        def __init__(self, name: str) -> None:
            self._name = name
            self._logger = FakeLogger()

        def declare_parameter(self, *_args, **_kwargs) -> None:
            return None

        def create_publisher(self, *_args, **_kwargs):
            return object()

        def get_name(self) -> str:
            return self._name

        def get_logger(self) -> FakeLogger:
            return self._logger

    class FakeQoSProfile:
        def __init__(self, **kwargs) -> None:
            self.kwargs = kwargs

    fake_rclpy = types.ModuleType("rclpy")
    fake_node = types.ModuleType("rclpy.node")
    fake_qos = types.ModuleType("rclpy.qos")
    fake_node.Node = FakeNode
    fake_qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL="transient_local")
    fake_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
    fake_qos.QoSProfile = FakeQoSProfile
    fake_qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="reliable")

    fake_nav_msgs = types.ModuleType("nav_msgs")
    fake_nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    fake_nav_msgs_msg.OccupancyGrid = type("OccupancyGrid", (), {})
    fake_std_msgs = types.ModuleType("std_msgs")
    fake_std_msgs_msg = types.ModuleType("std_msgs.msg")
    fake_std_msgs_msg.Header = type("Header", (), {})
    fake_builtin = types.ModuleType("builtin_interfaces")
    fake_builtin_msg = types.ModuleType("builtin_interfaces.msg")
    fake_builtin_msg.Time = type("Time", (), {})

    for name in (
        "scout_control.utils.grid_generator",
        "rclpy",
        "rclpy.node",
        "rclpy.qos",
        "nav_msgs",
        "nav_msgs.msg",
        "std_msgs",
        "std_msgs.msg",
        "builtin_interfaces",
        "builtin_interfaces.msg",
    ):
        monkeypatch.delitem(sys.modules, name, raising=False)
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", fake_node)
    monkeypatch.setitem(sys.modules, "rclpy.qos", fake_qos)
    monkeypatch.setitem(sys.modules, "nav_msgs", fake_nav_msgs)
    monkeypatch.setitem(sys.modules, "nav_msgs.msg", fake_nav_msgs_msg)
    monkeypatch.setitem(sys.modules, "std_msgs", fake_std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", fake_std_msgs_msg)
    monkeypatch.setitem(sys.modules, "builtin_interfaces", fake_builtin)
    monkeypatch.setitem(sys.modules, "builtin_interfaces.msg", fake_builtin_msg)

    grid_generator = importlib.import_module("scout_control.utils.grid_generator")
    calls: list[str] = []

    def fake_run(self) -> bool:
        calls.append(self.get_name())
        return True

    monkeypatch.setattr(grid_generator.GridGenerator, "_run", fake_run)

    node = grid_generator.GridGenerator()
    assert calls == []
    assert node.run() is True
    assert calls == ["grid_generator"]
    assert node.run() is False
    assert calls == ["grid_generator"]

from __future__ import annotations

import ast
from pathlib import Path


RUNTIME_PATH = (
    Path(__file__).resolve().parents[1]
    / "scout_control"
    / "core"
    / "obstacle_avoidance_runtime.py"
)


def _function_call_order(function_name: str) -> list[str]:
    tree = ast.parse(RUNTIME_PATH.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "ObstacleAvoidanceRuntime":
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == function_name:
                    class _CallCollector(ast.NodeVisitor):
                        def __init__(self) -> None:
                            self.calls: list[str] = []

                        def visit_Call(self, subnode: ast.Call) -> None:
                            func = subnode.func
                            if isinstance(func, ast.Attribute) and isinstance(func.value, ast.Name):
                                if func.value.id == "self":
                                    self.calls.append(func.attr)
                            self.generic_visit(subnode)

                    collector = _CallCollector()
                    collector.visit(item)
                    return collector.calls
    raise AssertionError(f"Function {function_name} not found")


def _function_source(function_name: str) -> str:
    tree = ast.parse(RUNTIME_PATH.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "ObstacleAvoidanceRuntime":
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == function_name:
                    return ast.get_source_segment(RUNTIME_PATH.read_text(encoding="utf-8"), item) or ""
    raise AssertionError(f"Function {function_name} not found")


def test_takeoff_requests_activation_before_publishing_setpoint() -> None:
    calls = _function_call_order("_do_takeoff")
    assert "_ensure_takeoff_activation" in calls
    assert "_publish_setpoint" in calls
    assert calls.index("_ensure_takeoff_activation") < calls.index("_publish_setpoint")


def test_takeoff_activation_contains_both_offboard_and_arm_requests() -> None:
    source = _function_source("_ensure_takeoff_activation")
    assert "_set_offboard_mode()" in source
    assert "_arm()" in source
    assert "_px4_offboard_enabled" in source
    assert "_px4_armed" in source


def test_idle_path_no_longer_sends_one_shot_arm_commands() -> None:
    source = _function_source("_do_idle")
    assert "_arm()" not in source
    assert "_set_offboard_mode()" not in source

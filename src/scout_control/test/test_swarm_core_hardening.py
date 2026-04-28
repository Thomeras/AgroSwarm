from __future__ import annotations

import ast
from pathlib import Path


CORE_DIR = Path(__file__).resolve().parents[1] / "scout_control" / "core"


def _module_tree(name: str) -> ast.Module:
    return ast.parse((CORE_DIR / name).read_text(encoding="utf-8"))


def _module_source(name: str) -> str:
    return (CORE_DIR / name).read_text(encoding="utf-8")


def _class_methods(tree: ast.Module, class_name: str) -> dict[str, ast.FunctionDef]:
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            return {
                item.name: item for item in node.body if isinstance(item, ast.FunctionDef)
            }
    raise AssertionError(f"class {class_name} not found")


def test_cell_data_recorder_uses_drone_count_and_topic_templates() -> None:
    source = _module_source("cell_data_recorder.py")

    assert 'declare_parameter("drone_count", 2)' in source
    assert 'declare_parameter("camera_topic_template", "")' in source
    assert 'declare_parameter("vehicle_position_topic_template", "")' in source
    assert "TelemetryHub(drone_id=idx)" not in source
    assert "range(self._drone_count)" in source
    assert "/px4_1/fmu/out/vehicle_local_position_v1" not in source
    assert 'Image, "/camera/image_raw"' not in source


def test_mission_launcher_timer_requests_shutdown_without_system_exit() -> None:
    tree = _module_tree("mission_launcher.py")
    source = _module_source("mission_launcher.py")
    methods = _class_methods(tree, "MissionLauncher")

    assert "_request_shutdown" in methods
    assert "raise SystemExit" not in source
    assert "except (KeyboardInterrupt, SystemExit)" not in source
    assert "rclpy.try_shutdown()" in ast.unparse(methods["_request_shutdown"])


def test_spray_controller_persists_log_with_atomic_replace() -> None:
    source = _module_source("spray_controller.py")
    tree = _module_tree("spray_controller.py")
    methods = _class_methods(tree, "SprayController")
    flush_body = ast.unparse(methods["_flush_log"])

    assert "import tempfile" in source
    assert "NamedTemporaryFile" in flush_body
    assert "os.fsync" in flush_body
    assert "os.replace" in flush_body
    assert 'open(SPRAY_LOG_FILE, "w")' not in source


def test_ml_interface_is_marked_as_tooling_placeholder() -> None:
    source = _module_source("ml_interface.py")

    assert "TOOLING_PLACEHOLDER = True" in source
    assert 'MODEL_MODE = "stub_tooling_placeholder"' in source
    assert '"model_mode": MODEL_MODE' in source
    assert "není produkční ML inference vrstva" in source

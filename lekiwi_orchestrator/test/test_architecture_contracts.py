# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Architecture contracts and dependency direction unit tests.

Verifies that the refactored package maintains strict acyclic, single-directional
dependency flow without circular or reverse coupling, conforms to a 100% flattened
package layout with zero subpackages, and keeps all modules under 1000 LOC.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parent.parent / "lekiwi_orchestrator"


def extract_imports(file_path: Path) -> set[str]:
    """Parse a python file AST and extract imported module names."""
    tree = ast.parse(file_path.read_text(encoding="utf-8"), filename=str(file_path))
    imports = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imports.add(alias.name)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module)
    return imports


def test_package_is_completely_flat():
    """Verify that lekiwi_orchestrator has no subdirectories (zero subpackages)."""
    subdirs = [
        d.name
        for d in PACKAGE_ROOT.iterdir()
        if d.is_dir() and d.name != "__pycache__" and not d.name.startswith(".")
    ]
    assert (
        len(subdirs) == 0
    ), f"lekiwi_orchestrator is not completely flat! Found subdirectories: {subdirs}"


def test_board_geometry_has_no_higher_layer_imports():
    """Verify board_geometry never imports from higher orchestrator layers."""
    geom_file = PACKAGE_ROOT / "board_geometry.py"
    assert geom_file.exists(), "board_geometry.py must exist at package root"
    imported = extract_imports(geom_file)
    for imp in imported:
        assert not imp.startswith(
            "lekiwi_orchestrator.motion"
        ), f"board_geometry violates architecture: imports {imp}"
        assert not imp.startswith(
            "lekiwi_orchestrator.move_sequencer"
        ), f"board_geometry violates architecture: imports {imp}"
        assert not imp.startswith(
            "lekiwi_orchestrator.orchestrator_node"
        ), f"board_geometry violates architecture: imports {imp}"
        assert not imp.startswith(
            "lekiwi_orchestrator.turn_workflow"
        ), f"board_geometry violates architecture: imports {imp}"
        assert not imp.startswith(
            "lekiwi_orchestrator.health_supervisor"
        ), f"board_geometry violates architecture: imports {imp}"


def test_mission_types_has_no_intra_package_imports():
    """Verify mission_types.py is a pure leaf domain model with zero intra-package dependencies."""
    types_file = PACKAGE_ROOT / "mission_types.py"
    assert types_file.exists(), "mission_types.py must exist at package root"
    imported = extract_imports(types_file)
    for imp in imported:
        assert not imp.startswith(
            "lekiwi_orchestrator."
        ), f"mission_types.py violates leaf domain architecture: imports {imp}"


def test_move_planner_has_no_sequencer_or_node_imports():
    """Verify move_planner.py never imports from move_sequencer, orchestrator_node, or turn_workflow."""
    planner_file = PACKAGE_ROOT / "move_planner.py"
    assert planner_file.exists(), "move_planner.py must exist at package root"
    imported = extract_imports(planner_file)
    for imp in imported:
        assert not imp.startswith(
            "lekiwi_orchestrator.move_sequencer"
        ), f"move_planner.py violates architecture: imports {imp}"
        assert not imp.startswith(
            "lekiwi_orchestrator.orchestrator_node"
        ), f"move_planner.py violates architecture: imports {imp}"
        assert not imp.startswith(
            "lekiwi_orchestrator.turn_workflow"
        ), f"move_planner.py violates architecture: imports {imp}"


def test_health_supervisor_has_no_workflow_or_sequencer_imports():
    """Verify health_supervisor.py does not depend on turn_workflow, move_sequencer, or node."""
    health_file = PACKAGE_ROOT / "health_supervisor.py"
    assert health_file.exists(), "health_supervisor.py must exist at package root"
    imported = extract_imports(health_file)
    for imp in imported:
        assert not imp.startswith(
            "lekiwi_orchestrator.turn_workflow"
        ), f"health_supervisor.py violates architecture: imports {imp}"
        assert not imp.startswith(
            "lekiwi_orchestrator.move_sequencer"
        ), f"health_supervisor.py violates architecture: imports {imp}"
        assert not imp.startswith(
            "lekiwi_orchestrator.orchestrator_node"
        ), f"health_supervisor.py violates architecture: imports {imp}"


def test_move_sequencer_has_no_node_or_turn_workflow_imports():
    """Verify move_sequencer.py does not depend on orchestrator_node or turn_workflow."""
    sequencer_file = PACKAGE_ROOT / "move_sequencer.py"
    assert sequencer_file.exists(), "move_sequencer.py must exist at package root"
    imported = extract_imports(sequencer_file)
    for imp in imported:
        assert not imp.startswith(
            "lekiwi_orchestrator.orchestrator_node"
        ), f"move_sequencer.py violates architecture: imports {imp}"
        assert not imp.startswith(
            "lekiwi_orchestrator.turn_workflow"
        ), f"move_sequencer.py violates architecture: imports {imp}"


def test_file_line_counts_under_budget():
    """Verify all Python source files in lekiwi_orchestrator are within 1000 lines."""
    for py_file in PACKAGE_ROOT.rglob("*.py"):
        if "__pycache__" in str(py_file):
            continue
        line_count = len(py_file.read_text(encoding="utf-8").splitlines())
        assert (
            line_count < 1000
        ), f"File {py_file.name} has {line_count} lines, exceeding 1000 LOC budget."


def test_zero_legacy_aliases_in_package():
    """Verify that all backward-compatible and legacy aliases have been completely purged."""
    import lekiwi_orchestrator

    legacy_symbols = [
        "MissionState",
        "OrchestratorConfig",
        "PerceptionContextCoordinator",
        "ActiveObservationNavigator",
        "ObservationNavigator",
        "ExecutionStage",
        "StagePipelineBuilder",
        "StageName",
        "StageBuilderFunc",
        "MovePipelineExecutor",
        "ActionDispatcherInterface",
        "RosActionDispatcher",
        "SimulatedActionDispatcher",
        "NodeHealthMonitor",
        "HealthMonitorConfig",
        "MissionStatusMarkerBuilder",
        "MissionWorkflowHost",
        "RefereeHandler",
        "MoveWorkflowCoordinator",
        "PostMoveWatchdog",
    ]
    for symbol in legacy_symbols:
        assert (
            symbol not in lekiwi_orchestrator.__all__
        ), f"Legacy alias '{symbol}' still exported in lekiwi_orchestrator.__all__"
        assert not hasattr(
            lekiwi_orchestrator, symbol
        ), f"Legacy alias '{symbol}' still exists on lekiwi_orchestrator module"


def test_parameters_invalid_readiness_timeout():
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from lekiwi_orchestrator.parameters import OrchestratorParameters

    if not rclpy.ok():
        rclpy.init()
    test_node = Node(
        "test_params_node",
        parameter_overrides=[Parameter("readiness_timeout_sec", value=-1.0)],
    )
    try:
        with pytest.raises(
            ValueError, match="readiness_timeout_sec must be finite and positive"
        ):
            OrchestratorParameters(test_node)
    finally:
        test_node.destroy_node()


def test_move_step_to_ros_goal_missing_interface(monkeypatch):
    import lekiwi_orchestrator.move_planner as mp
    from geometry_msgs.msg import Point

    monkeypatch.setattr(mp, "ExecuteChessMove", None)
    step = mp.MoveStep(
        name="TEST",
        target_pose=None,
        instruction="test",
        from_square="e2",
        to_square="e4",
        is_capture=False,
        pick_point=Point(),
        place_point=Point(),
    )
    with pytest.raises(
        RuntimeError, match="ExecuteChessMove action interface is unavailable"
    ):
        step.to_ros_goal("board")

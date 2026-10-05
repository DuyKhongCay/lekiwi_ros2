# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""LeKiwi Orchestration package for autonomous chess mobile manipulation."""

from lekiwi_orchestrator.board_geometry import (
    RankedViewpoint,
    compute_radial_entry_pose,
    is_on_standoff_circle,
    normalize_angle,
    rank_by_azimuth,
    yaw_to_quaternion,
)
from lekiwi_orchestrator.fsm import (
    ALLOWED_MISSION_TRANSITIONS,
    ALLOWED_MOTION_TRANSITIONS,
    ALLOWED_PERCEPTION_TRANSITIONS,
    MISSION_STATE_NAMES,
    MOTION_STATE_NAMES,
    PERCEPTION_CONTEXT_NAMES,
    MacroMissionState,
    MotionExecutionState,
    PerceptionContext,
    is_mission_transition_allowed,
    is_motion_transition_allowed,
    is_perception_transition_allowed,
)
from lekiwi_orchestrator.health_supervisor import (
    HealthSupervisor,
    HealthSupervisorConfig,
    OrchestratorStateSnapshot,
    StatusMarkerBuilder,
    VisualizerConfig,
)
from lekiwi_orchestrator.mission_types import (
    ActionResult,
    ChessMoveGoal,
    ObservationIntent,
)
from lekiwi_orchestrator.motion_client import (
    FakeMotionClient,
    FeasibilityClient,
    ManipulationClient,
    MotionClient,
    NavigationClient,
    RosMotionClient,
)
from lekiwi_orchestrator.move_planner import (
    MovePlanBuilder,
    MoveStep,
    StepKind,
)
from lekiwi_orchestrator.move_sequencer import MoveSequencer
from lekiwi_orchestrator.obs_navigator import ObsNavigator
from lekiwi_orchestrator.orchestrator_node import (
    ChessMissionOrchestrator,
    OrchestratorSubsystems,
)
from lekiwi_orchestrator.parameters import OrchestratorParameters
from lekiwi_orchestrator.perception_context import PerceptionContextManager
from lekiwi_orchestrator.turn_workflow import (
    GameStatusHandler,
    MoveWorkflow,
    PostMoveVerifier,
    TurnHost,
)

__all__ = [
    # FSM
    "ALLOWED_MISSION_TRANSITIONS",
    "ALLOWED_MOTION_TRANSITIONS",
    "ALLOWED_PERCEPTION_TRANSITIONS",
    "MISSION_STATE_NAMES",
    "MOTION_STATE_NAMES",
    "PERCEPTION_CONTEXT_NAMES",
    "MacroMissionState",
    "MotionExecutionState",
    "PerceptionContext",
    "is_mission_transition_allowed",
    "is_motion_transition_allowed",
    "is_perception_transition_allowed",
    # Geometry
    "RankedViewpoint",
    "compute_radial_entry_pose",
    "is_on_standoff_circle",
    "normalize_angle",
    "rank_by_azimuth",
    "yaw_to_quaternion",
    # Mission Types
    "ActionResult",
    "ChessMoveGoal",
    "ObservationIntent",
    # Parameters
    "OrchestratorParameters",
    # Perception Context
    "PerceptionContextManager",
    # Motion Clients
    "MotionClient",
    "FeasibilityClient",
    "NavigationClient",
    "ManipulationClient",
    "RosMotionClient",
    "FakeMotionClient",
    # Move Planner & Sequencer
    "StepKind",
    "MoveStep",
    "MovePlanBuilder",
    "MoveSequencer",
    # Observation Navigator
    "ObsNavigator",
    # Health Supervisor
    "HealthSupervisor",
    "HealthSupervisorConfig",
    "StatusMarkerBuilder",
    "OrchestratorStateSnapshot",
    "VisualizerConfig",
    # Turn Workflow
    "TurnHost",
    "GameStatusHandler",
    "MoveWorkflow",
    "PostMoveVerifier",
    # Orchestrator Node
    "ChessMissionOrchestrator",
    "OrchestratorSubsystems",
]

# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""LeKiwi Orchestration package for autonomous chess mobile manipulation.

Facade exposing all symbols across the 4 architectural layers:
- core (FSM, mission models, parameters, utilities)
- planning (board geometry, move decomposition, active observation navigation)
- execution (motion clients, move sequencers, turn workflow hosts)
- supervision (health supervisor, perception lifecycle gating, orchestrator node)
"""

from lekiwi_orchestrator.core.fsm import (
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
from lekiwi_orchestrator.core.mission_types import (
    ActionResult,
    ChessMoveGoal,
    ObservationIntent,
)
from lekiwi_orchestrator.core.parameters import OrchestratorParameters
from lekiwi_orchestrator.core.utils import (
    create_pose2d_stamped,
    get_header_stamp_sec,
    get_node_now_sec,
    normalize_angle,
    yaw_to_quaternion,
)
from lekiwi_orchestrator.execution.motion_client import (
    FakeMotionClient,
    FeasibilityClient,
    ManipulationClient,
    MotionClient,
    NavigationClient,
    RosMotionClient,
)
from lekiwi_orchestrator.execution.move_sequencer import MoveSequencer
from lekiwi_orchestrator.execution.turn_workflow import (
    GameStatusHandler,
    MoveWorkflow,
    PostMoveVerifier,
    TurnHost,
)
from lekiwi_orchestrator.planning.board_geometry import (
    RankedViewpoint,
    rank_by_azimuth,
)
from lekiwi_orchestrator.planning.move_planner import (
    MovePlanBuilder,
    MoveStep,
    StepKind,
)
from lekiwi_orchestrator.planning.obs_navigator import ObsNavigator
from lekiwi_orchestrator.supervision.health_supervisor import (
    HealthSupervisor,
    HealthSupervisorConfig,
    OrchestratorStateSnapshot,
    StatusMarkerBuilder,
    VisualizerConfig,
)
from lekiwi_orchestrator.supervision.orchestrator_node import (
    ChessMissionOrchestrator,
    OrchestratorSubsystems,
    main,
)
from lekiwi_orchestrator.supervision.perception_context import PerceptionContextManager

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
    "rank_by_azimuth",
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
    "main",
    # Utilities
    "get_node_now_sec",
    "get_header_stamp_sec",
    "normalize_angle",
    "yaw_to_quaternion",
    "create_pose2d_stamped",
]

# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Move execution steps and plan builder.

Maps workspace feasibility plans (ZERO_NAV, SINGLE_BASE, DUAL_BASE, etc.)
into ordered sequences of atomic MoveSteps.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from typing import Any, ClassVar

from geometry_msgs.msg import Point, PoseStamped
from lekiwi_interfaces.srv import CheckMoveFeasibility

from lekiwi_orchestrator.fsm import MotionExecutionState
from lekiwi_orchestrator.mission_types import ChessMoveGoal

try:
    from lekiwi_interfaces.action import ExecuteChessMove
except ImportError:
    ExecuteChessMove = None  # type: ignore[assignment, misc]


class StepKind(str, Enum):
    """Enumeration of atomic move execution step types."""

    CLEAR = "CLEAR"
    PICK = "PICK"
    PLACE = "PLACE"
    MOVE = "MOVE"
    OBSERVATION = "OBSERVATION"

    def __str__(self) -> str:
        return self.value



@dataclass
class MoveStep:
    """Represents a single atomic operation in a multi-step chess move plan."""

    name: StepKind | str
    target_pose: PoseStamped | None
    instruction: str
    from_square: str
    to_square: str
    is_capture: bool
    pick_point: Point
    place_point: Point
    motion_state: MotionExecutionState = MotionExecutionState.IDLE
    nav_motion_state: MotionExecutionState | None = None

    def to_ros_goal(
        self,
        board_frame: str,
        feasibility_resp: CheckMoveFeasibility.Response | None = None,
    ) -> Any:
        """Assemble ExecuteChessMove.Goal message with points and IK hints."""
        if ExecuteChessMove is None:
            raise RuntimeError("ExecuteChessMove action interface is unavailable")

        goal = ExecuteChessMove.Goal()
        goal.instruction = self.instruction
        goal.from_square = self.from_square
        goal.to_square = self.to_square
        goal.is_capture = self.is_capture
        goal.pick_point = self.pick_point
        goal.place_point = self.place_point
        goal.target_frame = board_frame

        if feasibility_resp is not None:
            if hasattr(feasibility_resp, "pick_ik_solution") and getattr(
                feasibility_resp.pick_ik_solution, "name", None
            ):
                goal.pick_ik_hint = feasibility_resp.pick_ik_solution
            if hasattr(feasibility_resp, "place_ik_solution") and getattr(
                feasibility_resp.place_ik_solution, "name", None
            ):
                goal.place_ik_hint = feasibility_resp.place_ik_solution
        return goal


def _clear_step(
    from_square: str,
    pick_point: Point,
    target_pose: PoseStamped | None = None,
    nav_state: MotionExecutionState | None = None,
) -> MoveStep:
    return MoveStep(
        name=StepKind.CLEAR,
        target_pose=target_pose,
        instruction=f"Clear {from_square} to onboard bin",
        from_square=from_square,
        to_square="",
        is_capture=True,
        pick_point=pick_point,
        place_point=Point(),
        motion_state=MotionExecutionState.CLEARING_PIECE,
        nav_motion_state=nav_state,
    )


def _pick_step(
    from_square: str,
    pick_point: Point,
    target_pose: PoseStamped | None = None,
    nav_state: MotionExecutionState | None = None,
) -> MoveStep:
    return MoveStep(
        name=StepKind.PICK,
        target_pose=target_pose,
        instruction=f"Pick {from_square}",
        from_square=from_square,
        to_square="",
        is_capture=False,
        pick_point=pick_point,
        place_point=Point(),
        motion_state=MotionExecutionState.PICKING_PIECE,
        nav_motion_state=nav_state,
    )


def _place_step(
    to_square: str,
    place_point: Point,
    target_pose: PoseStamped | None = None,
    nav_state: MotionExecutionState | None = None,
) -> MoveStep:
    return MoveStep(
        name=StepKind.PLACE,
        target_pose=target_pose,
        instruction=f"Place {to_square}",
        from_square="",
        to_square=to_square,
        is_capture=False,
        pick_point=Point(),
        place_point=place_point,
        motion_state=MotionExecutionState.PLACING_PIECE,
        nav_motion_state=nav_state,
    )


def _move_step(
    from_square: str,
    to_square: str,
    pick_point: Point,
    place_point: Point,
    target_pose: PoseStamped | None = None,
    is_capture: bool = False,
    nav_state: MotionExecutionState | None = None,
) -> MoveStep:
    return MoveStep(
        name=StepKind.MOVE,
        target_pose=target_pose,
        instruction=f"Pick {from_square}, place {to_square}",
        from_square=from_square,
        to_square=to_square,
        is_capture=is_capture,
        pick_point=pick_point,
        place_point=place_point,
        motion_state=MotionExecutionState.PICKING_PIECE,
        nav_motion_state=nav_state,
    )


def _observation_step(observation_pose: PoseStamped) -> MoveStep:
    return MoveStep(
        name=StepKind.OBSERVATION,
        target_pose=observation_pose,
        instruction="Retreat base to primary observation standoff facing chessboard",
        from_square="",
        to_square="",
        is_capture=False,
        pick_point=Point(),
        place_point=Point(),
        motion_state=MotionExecutionState.IDLE,
        nav_motion_state=MotionExecutionState.NAV_TO_OBS,
    )


StepBuilderFunc = Callable[
    [CheckMoveFeasibility.Response, ChessMoveGoal], list[MoveStep]
]


class MovePlanBuilder:
    """Transforms feasibility calculation results into an ordered list of MoveSteps."""

    _BUILDERS: ClassVar[dict[int, StepBuilderFunc]] = {}

    @classmethod
    def register_builder(cls, plan_type: int, builder: StepBuilderFunc) -> None:
        """Register or override a step builder for a specific plan type."""
        cls._BUILDERS[plan_type] = builder

    @classmethod
    def build_steps(
        cls,
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
        observation_pose: PoseStamped | None = None,
    ) -> list[MoveStep]:
        """Entry point for building move steps across all feasibility plan types via Strategy Map."""
        builder = cls._BUILDERS.get(resp.plan_type, cls._build_fallback)
        steps = list(builder(resp, details))
        if observation_pose is not None:
            steps.append(_observation_step(observation_pose))
        return steps

    # Backward compatibility alias
    build_stages = build_steps

    @staticmethod
    def _build_zero_nav(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[MoveStep]:
        return [
            _move_step(
                from_square=details.from_square,
                to_square=details.to_square,
                pick_point=resp.pick_point,
                place_point=resp.place_point,
            )
        ]

    @staticmethod
    def _build_single_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[MoveStep]:
        return [
            _move_step(
                from_square=details.from_square,
                to_square=details.to_square,
                pick_point=resp.pick_point,
                place_point=resp.place_point,
                target_pose=resp.pick_base_pose,
                nav_state=MotionExecutionState.NAV_TO_PICK,
            )
        ]

    @staticmethod
    def _build_dual_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[MoveStep]:
        return [
            _pick_step(
                from_square=details.from_square,
                pick_point=resp.pick_point,
                target_pose=resp.pick_base_pose,
                nav_state=MotionExecutionState.NAV_TO_PICK,
            ),
            _place_step(
                to_square=details.to_square,
                place_point=resp.place_point,
                target_pose=resp.place_base_pose,
                nav_state=MotionExecutionState.NAV_TO_PLACE,
            ),
        ]

    @staticmethod
    def _build_capture_zero_nav(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[MoveStep]:
        cap_sq = details.captured_square or details.to_square
        return [
            _clear_step(
                from_square=cap_sq,
                pick_point=resp.clear_point,
            ),
            _move_step(
                from_square=details.from_square,
                to_square=details.to_square,
                pick_point=resp.pick_point,
                place_point=resp.place_point,
            ),
        ]

    @staticmethod
    def _build_capture_single_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[MoveStep]:
        cap_sq = details.captured_square or details.to_square
        return [
            _clear_step(
                from_square=cap_sq,
                pick_point=resp.clear_point,
                target_pose=resp.clear_base_pose,
                nav_state=MotionExecutionState.NAV_TO_CLEAR,
            ),
            _move_step(
                from_square=details.from_square,
                to_square=details.to_square,
                pick_point=resp.pick_point,
                place_point=resp.place_point,
            ),
        ]

    @staticmethod
    def _build_capture_dual_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[MoveStep]:
        cap_sq = details.captured_square or details.to_square
        dx = resp.clear_base_pose.pose.position.x - resp.pick_base_pose.pose.position.x
        dy = resp.clear_base_pose.pose.position.y - resp.pick_base_pose.pose.position.y
        clear_same_as_pick = math.hypot(dx, dy) < 0.05

        return [
            _clear_step(
                from_square=cap_sq,
                pick_point=resp.clear_point,
                target_pose=resp.clear_base_pose,
                nav_state=MotionExecutionState.NAV_TO_CLEAR,
            ),
            _pick_step(
                from_square=details.from_square,
                pick_point=resp.pick_point,
                target_pose=None if clear_same_as_pick else resp.pick_base_pose,
                nav_state=(
                    None if clear_same_as_pick else MotionExecutionState.NAV_TO_PICK
                ),
            ),
            _place_step(
                to_square=details.to_square,
                place_point=resp.place_point,
                target_pose=resp.place_base_pose,
                nav_state=MotionExecutionState.NAV_TO_PLACE,
            ),
        ]

    @staticmethod
    def _build_capture_triple_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[MoveStep]:
        cap_sq = details.captured_square or details.to_square
        return [
            _clear_step(
                from_square=cap_sq,
                pick_point=resp.clear_point,
                target_pose=resp.clear_base_pose,
                nav_state=MotionExecutionState.NAV_TO_CLEAR,
            ),
            _pick_step(
                from_square=details.from_square,
                pick_point=resp.pick_point,
                target_pose=resp.pick_base_pose,
                nav_state=MotionExecutionState.NAV_TO_PICK,
            ),
            _place_step(
                to_square=details.to_square,
                place_point=resp.place_point,
                target_pose=resp.place_base_pose,
                nav_state=MotionExecutionState.NAV_TO_PLACE,
            ),
        ]

    @staticmethod
    def _build_fallback(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[MoveStep]:
        target_pose = getattr(resp, "pick_base_pose", None)
        return [
            _move_step(
                from_square=details.from_square,
                to_square=details.to_square,
                pick_point=getattr(resp, "pick_point", Point()),
                place_point=getattr(resp, "place_point", Point()),
                target_pose=target_pose,
                is_capture=details.is_capture,
                nav_state=(MotionExecutionState.NAV_TO_PICK if target_pose else None),
            )
        ]


# Default registry mapping feasibility plan types to specialized step builders
MovePlanBuilder._BUILDERS = {
    CheckMoveFeasibility.Response.PLAN_ZERO_NAV: MovePlanBuilder._build_zero_nav,
    CheckMoveFeasibility.Response.PLAN_SINGLE_BASE: MovePlanBuilder._build_single_base,
    CheckMoveFeasibility.Response.PLAN_DUAL_BASE: MovePlanBuilder._build_dual_base,
    CheckMoveFeasibility.Response.PLAN_CAPTURE_ZERO_NAV: MovePlanBuilder._build_capture_zero_nav,
    CheckMoveFeasibility.Response.PLAN_CAPTURE_SINGLE_BASE: MovePlanBuilder._build_capture_single_base,
    CheckMoveFeasibility.Response.PLAN_CAPTURE_DUAL_BASE: MovePlanBuilder._build_capture_dual_base,
    CheckMoveFeasibility.Response.PLAN_CAPTURE_TRIPLE_BASE: MovePlanBuilder._build_capture_triple_base,
}


__all__ = [
    "StepKind",
    "MoveStep",
    "MovePlanBuilder",
    "StepBuilderFunc",
]

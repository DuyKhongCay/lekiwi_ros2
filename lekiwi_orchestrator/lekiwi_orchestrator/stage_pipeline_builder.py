# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Pure domain builder translating CheckMoveFeasibility responses into atomic ExecutionStages.
Decouples trajectory stage choreography from the ROS 2 orchestrator node.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from geometry_msgs.msg import Point, PoseStamped
from lekiwi_interfaces.srv import CheckMoveFeasibility


@dataclass(frozen=True)
class ChessMoveGoal:
    """Lightweight mission-level move description."""

    uci: str
    from_square: str
    to_square: str
    promotion: str | None = None
    is_capture: bool = False
    captured_square: str = ""
    castling_rook_from: str = ""
    castling_rook_to: str = ""


@dataclass
class ExecutionStage:
    """Represents a single atomic operation in a multi-stage chess move."""

    name: str  # "CLEAR", "PICK", "PLACE", "MOVE"
    target_pose: PoseStamped | None
    instruction: str
    from_square: str
    to_square: str
    is_capture: bool
    pick_point: Point
    place_point: Point


class StagePipelineBuilder:
    """Transforms feasibility calculation results into an ordered list of ExecutionStages."""

    @staticmethod
    def build_stages(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        """Entry point for building stages across all feasibility plan types."""
        plan_type = resp.plan_type
        if plan_type == CheckMoveFeasibility.Response.PLAN_ZERO_NAV:
            return StagePipelineBuilder._build_zero_nav(resp, details)
        if plan_type == CheckMoveFeasibility.Response.PLAN_SINGLE_BASE:
            return StagePipelineBuilder._build_single_base(resp, details)
        if plan_type == CheckMoveFeasibility.Response.PLAN_DUAL_BASE:
            return StagePipelineBuilder._build_dual_base(resp, details)
        if plan_type == CheckMoveFeasibility.Response.PLAN_CAPTURE_ZERO_NAV:
            return StagePipelineBuilder._build_capture_zero_nav(resp, details)
        if plan_type == CheckMoveFeasibility.Response.PLAN_CAPTURE_SINGLE_BASE:
            return StagePipelineBuilder._build_capture_single_base(resp, details)
        if plan_type == CheckMoveFeasibility.Response.PLAN_CAPTURE_DUAL_BASE:
            return StagePipelineBuilder._build_capture_dual_base(resp, details)
        if plan_type == CheckMoveFeasibility.Response.PLAN_CAPTURE_TRIPLE_BASE:
            return StagePipelineBuilder._build_capture_triple_base(resp, details)
        return StagePipelineBuilder._build_fallback(resp, details)

    @staticmethod
    def _build_zero_nav(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        return [
            ExecutionStage(
                name="MOVE",
                target_pose=None,
                instruction=f"Pick {details.from_square}, place {details.to_square}",
                from_square=details.from_square,
                to_square=details.to_square,
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=resp.place_point,
            )
        ]

    @staticmethod
    def _build_single_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        return [
            ExecutionStage(
                name="MOVE",
                target_pose=resp.pick_base_pose,
                instruction=f"Pick {details.from_square}, place {details.to_square}",
                from_square=details.from_square,
                to_square=details.to_square,
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=resp.place_point,
            )
        ]

    @staticmethod
    def _build_dual_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        return [
            ExecutionStage(
                name="PICK",
                target_pose=resp.pick_base_pose,
                instruction=f"Pick {details.from_square}",
                from_square=details.from_square,
                to_square="",
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=Point(),
            ),
            ExecutionStage(
                name="PLACE",
                target_pose=resp.place_base_pose,
                instruction=f"Place {details.to_square}",
                from_square="",
                to_square=details.to_square,
                is_capture=False,
                pick_point=Point(),
                place_point=resp.place_point,
            ),
        ]

    @staticmethod
    def _build_capture_zero_nav(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        cap_sq = details.captured_square or details.to_square
        return [
            ExecutionStage(
                name="CLEAR",
                target_pose=None,
                instruction=f"Clear {cap_sq} to onboard bin",
                from_square=cap_sq,
                to_square="",
                is_capture=True,
                pick_point=resp.clear_point,
                place_point=Point(),
            ),
            ExecutionStage(
                name="MOVE",
                target_pose=None,
                instruction=f"Pick {details.from_square}, place {details.to_square}",
                from_square=details.from_square,
                to_square=details.to_square,
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=resp.place_point,
            ),
        ]

    @staticmethod
    def _build_capture_single_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        cap_sq = details.captured_square or details.to_square
        return [
            ExecutionStage(
                name="CLEAR",
                target_pose=resp.clear_base_pose,
                instruction=f"Clear {cap_sq} to onboard bin",
                from_square=cap_sq,
                to_square="",
                is_capture=True,
                pick_point=resp.clear_point,
                place_point=Point(),
            ),
            ExecutionStage(
                name="MOVE",
                target_pose=None,
                instruction=f"Pick {details.from_square}, place {details.to_square}",
                from_square=details.from_square,
                to_square=details.to_square,
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=resp.place_point,
            ),
        ]

    @staticmethod
    def _build_capture_dual_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        cap_sq = details.captured_square or details.to_square
        dx = resp.clear_base_pose.pose.position.x - resp.pick_base_pose.pose.position.x
        dy = resp.clear_base_pose.pose.position.y - resp.pick_base_pose.pose.position.y
        clear_same_as_pick = math.hypot(dx, dy) < 0.05

        return [
            ExecutionStage(
                name="CLEAR",
                target_pose=resp.clear_base_pose,
                instruction=f"Clear {cap_sq} to onboard bin",
                from_square=cap_sq,
                to_square="",
                is_capture=True,
                pick_point=resp.clear_point,
                place_point=Point(),
            ),
            ExecutionStage(
                name="PICK",
                target_pose=None if clear_same_as_pick else resp.pick_base_pose,
                instruction=f"Pick {details.from_square}",
                from_square=details.from_square,
                to_square="",
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=Point(),
            ),
            ExecutionStage(
                name="PLACE",
                target_pose=resp.place_base_pose,
                instruction=f"Place {details.to_square}",
                from_square="",
                to_square=details.to_square,
                is_capture=False,
                pick_point=Point(),
                place_point=resp.place_point,
            ),
        ]

    @staticmethod
    def _build_capture_triple_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        cap_sq = details.captured_square or details.to_square
        return [
            ExecutionStage(
                name="CLEAR",
                target_pose=resp.clear_base_pose,
                instruction=f"Clear {cap_sq} to onboard bin",
                from_square=cap_sq,
                to_square="",
                is_capture=True,
                pick_point=resp.clear_point,
                place_point=Point(),
            ),
            ExecutionStage(
                name="PICK",
                target_pose=resp.pick_base_pose,
                instruction=f"Pick {details.from_square}",
                from_square=details.from_square,
                to_square="",
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=Point(),
            ),
            ExecutionStage(
                name="PLACE",
                target_pose=resp.place_base_pose,
                instruction=f"Place {details.to_square}",
                from_square="",
                to_square=details.to_square,
                is_capture=False,
                pick_point=Point(),
                place_point=resp.place_point,
            ),
        ]

    @staticmethod
    def _build_fallback(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        return [
            ExecutionStage(
                name="MOVE",
                target_pose=resp.pick_base_pose,
                instruction=f"Pick {details.from_square}, place {details.to_square}",
                from_square=details.from_square,
                to_square=details.to_square,
                is_capture=details.is_capture,
                pick_point=getattr(resp, "pick_point", Point()),
                place_point=getattr(resp, "place_point", Point()),
            )
        ]

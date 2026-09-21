"""Launch LeKiwi control subsystem nodes and component container."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import (
    ComposableNodeContainer,
    LoadComposableNodes,
)
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start control component container and load composable nodes from control.yaml."""
    control_config = PathJoinSubstitution(
        [FindPackageShare("lekiwi_bringup"), "config", "control", "control.yaml"]
    )
    clock = {
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool
        )
    }

    # Component container for zero-copy intra-process communication
    control_container = ComposableNodeContainer(
        name="lekiwi_control_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[],
        output="screen",
    )

    # Dynamic loading of readiness check components into container when enabled
    load_readiness_nodes = LoadComposableNodes(
        target_container="lekiwi_control_container",
        composable_node_descriptions=[
            ComposableNode(
                package="lekiwi_control",
                plugin="lekiwi_control::WorkspaceCheckerNode",
                name="workspace_checker",
                parameters=[control_config, clock],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="lekiwi_control",
                plugin="lekiwi_control::TfGatekeeperNode",
                name="tf_gatekeeper_node",
                parameters=[control_config, clock],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_readiness_checks",
                default_value="true",
                description="Start both TF and workspace readiness checks",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            control_container,
            GroupAction(
                [load_readiness_nodes],
                condition=IfCondition(LaunchConfiguration("enable_readiness_checks")),
            ),
        ]
    )

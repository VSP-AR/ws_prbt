from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use simulation time",
    )

    # Build MoveIt config
    moveit_config = MoveItConfigsBuilder(
        "prbt_cell", package_name="prbt_cell_moveit_config"
    ).to_moveit_configs()

    move_group_ld = generate_move_group_launch(moveit_config)


    return LaunchDescription(
        [
            use_sim_time_arg,
            SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        ]
        + list(move_group_ld.entities)
    )

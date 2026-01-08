from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "-0.0148762",
                    "-0.0595541",
                    "0.0356587",
                    "0.489564",
                    "-0.493393",
                    "0.504213",
                    "0.512503",
                    "prbt_tool0",
                    "camera_link",
                ],
                name="camera_link_broadcaster",
            )
        ]
    )

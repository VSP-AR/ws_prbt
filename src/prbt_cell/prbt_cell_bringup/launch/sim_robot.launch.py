import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # -----------------------------
    # Launch arguments
    # -----------------------------
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use simulation time",
    )

    world = DeclareLaunchArgument(
        "world",
        default_value="warehouse",
        description="Simulation world",
    )

    # -----------------------------
    # File paths
    # -----------------------------
    prbt_xacro_file = os.path.join(
        get_package_share_directory("prbt_cell_description"),
        "urdf",
        "prbt_cell.urdf.xacro",
    )

    rviz_file = os.path.join(
        get_package_share_directory("prbt_cell_description"),
        "launch",
        "basic.rviz",
    )

    world_file = os.path.join(
        get_package_share_directory("prbt_cell_bringup"),
        "launch",
        "world.sdf",
    )

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    prbt_cell_description_pkg = get_package_share_directory("prbt_cell_description")

    prbt_cell_description_meshes = PathJoinSubstitution(
        [get_package_share_directory("prbt_cell_description"), "meshes"]
    )

    prbt_robot_support_meshes = PathJoinSubstitution(
        [get_package_share_directory("prbt_robot_support"), "meshes"]
    )

    gz_plugin_path = PathJoinSubstitution([prbt_cell_description_pkg, "plugins"])

    controller_config = PathJoinSubstitution(
        [
            get_package_share_directory("prbt_robot_support"),
            "config",
            "prbt_ros2_control.yaml",
        ]
    )

    # -----------------------------
    # Robot description
    # -----------------------------
    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                prbt_xacro_file,
                " ",
                "sim:=true",
                " ",
                "simulation_controllers:=",
                controller_config,
            ]
        ),
        value_type=str,
    )

    # -----------------------------
    # Nodes
    # -----------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
    )

    
    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments={"gz_args": f"{world_file} -r -v 4"}.items(),
    )

    
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

   
    bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "rgbd_camera/image",
            "rgbd_camera/depth_image",
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "prbt",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

  
    prbt_joint_loop = Node(
        package="prbt_cell_bringup",
        executable="prbt_joint_loop.py",
        name="prbt_joint_loop_node",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "poses_yaml": "config/joint_poses.yaml",
            "action_name": "/arm_controller/follow_joint_trajectory",
            "move_time": 4.0,
            "hold_time": 1.0,
            "ping_pong": False,
        }],
    )


    # -----------------------------
    # Launch description
    # -----------------------------
    return LaunchDescription(
        [
            use_sim_time,
            world,

         
            SetParameter(
                name="use_sim_time",
                value=LaunchConfiguration("use_sim_time"),
            ),

           
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                ":".join(
                    [
                        str(prbt_cell_description_meshes),
                        str(prbt_robot_support_meshes),
                    ]
                ),
            ),
            SetEnvironmentVariable(
                "GZ_SIM_PLUGIN_PATH",
                gz_plugin_path,
            ),

           
            robot_state_publisher,
            rviz2,
            gazebo,
            clock_bridge,
            bridge,
            spawn_robot,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            prbt_joint_loop,
        ]
    )

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _load_file(path):
    with open(path, "r", encoding="utf-8") as file_handle:
        return file_handle.read()


def _launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "True"
    urdf_path = LaunchConfiguration("urdf").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)

    robot_description = _load_file(urdf_path)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            "my_robot",
            "-file",
            urdf_path,
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.2",
        ],
    )

    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"],
        output="screen",
    )

    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return [
        robot_state_publisher,
        spawn_entity,
        static_map_to_odom,
        rviz_cmd,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("navegacion_gps")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    gazebo_share = get_package_share_directory("gazebo_ros")

    default_urdf = os.path.join(pkg_share, "urdf", "my_robot.urdf")
    default_rviz = os.path.join(pkg_share, "config", "rviz_nav2_full.rviz")
    default_params = os.path.join(pkg_share, "config", "nav2_no_map_params.yaml")
    default_world = os.path.join(gazebo_share, "worlds", "empty.world")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    params_file = LaunchConfiguration("params_file")
    use_ackermann_converter = LaunchConfiguration("use_ackermann_converter")

    configured_params = RewrittenYaml(
        source_file=params_file, root_key="", param_rewrites="", convert_types=True
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world file",
    )
    declare_urdf_cmd = DeclareLaunchArgument(
        "urdf",
        default_value=default_urdf,
        description="URDF file to spawn",
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Nav2 parameters file",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to start RVIZ",
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Path to the RViz config file",
    )
    declare_use_ackermann_converter_cmd = DeclareLaunchArgument(
        "use_ackermann_converter",
        default_value="False",
        description="Whether to convert /cmd_vel to Ackermann commands",
    )

    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            world,
            "-slibgazebo_ros_init.so",
            "-slibgazebo_ros_factory.so",
        ],
        output="screen",
    )

    gazebo_client = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    ackermann_converter_cmd = Node(
        package="navegacion_gps",
        executable="twist_to_ackermann",
        name="twist_to_ackermann",
        output="screen",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/ackermann_cmd",
                "wheelbase": 1.0,
                "steering_limit": 0.6,
            }
        ],
        condition=IfCondition(use_ackermann_converter),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_urdf_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_ackermann_converter_cmd)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(OpaqueFunction(function=_launch_setup))
    ld.add_action(navigation2_cmd)
    ld.add_action(ackermann_converter_cmd)
    return ld

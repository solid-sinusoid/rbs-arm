from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

from robot_builder.external.ros2_control import ControllerManager
from robot_builder.parser.urdf import URDF_parser
from rbs_utils.launch import load_xacro_args


def launch_setup(context, *args, **kwargs):
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    description_package = LaunchConfiguration("description_package").perform(context)
    description_file = LaunchConfiguration("description_file").perform(context)
    hardware = LaunchConfiguration("hardware").perform(context)


    description_package_abs_path = get_package_share_directory(
        description_package
    )

    controllers_file = os.path.join(
        description_package_abs_path, "config", "controllers.yaml"
    )

    xacro_file = os.path.join(
        description_package_abs_path,
        "urdf",
        description_file
    )

    xacro_config_file = os.path.join(
        description_package_abs_path, "urdf", "xacro_args.yaml"
    )


    mappings_data = load_xacro_args(xacro_config_file, locals())

    robot_description_doc = xacro.process_file(xacro_file, mappings=mappings_data)

    

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "view_robot.rviz"]
    )
    robot_description_content = robot_description_doc.toprettyxml(indent="  ")
    robot_description = {"robot_description": robot_description_content}

    # with open(f"rbs_g_arm0_6.urdf", 'w') as xfile:
    #     xfile.write(robot_description_content)
    #     xfile.close()


    robot = URDF_parser.load_string(
        robot_description_content, ee_link_name="ee_link"
    )
    ControllerManager.save_to_yaml(
        robot, description_package_abs_path, "controllers.yaml"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/controller_manager",
        ],
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c", "/controller_manager",
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                            'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r ', "empty.sdf"])],
    )
    # Spawn robot
    gazebo_spawn_robot = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-name', "arm0",
            '-x', '0.0',
            '-z', '0.0',
            '-y', '0.0',
            '-topic', "/robot_description"
        ],
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    nodes_to_start = [
        joint_state_broadcaster_spawner,
        joint_trajectory_controller,
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        gazebo_spawn_robot,
        clock_bridge
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="rbs_arm",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="rbs_arm_modular.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='',
            description="tf_prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware",
            default_value='',
            # choices=["mock", "gazebo"],
            description="The name of hardware", 
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

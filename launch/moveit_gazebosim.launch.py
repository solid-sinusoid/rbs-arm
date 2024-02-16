from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

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
            "prefix",
            default_value='',
            description="Prefix of the joint names, useful for \
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
    prefix = LaunchConfiguration("prefix")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    hardware = LaunchConfiguration("hardware")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "view_robot.rviz"]
    )
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("rbs_arm"), "config", "rbs_arm_controllers_gazebosim.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "gripper_name:=", "", " ",
            "prefix:=", prefix, " ",
            "hardware:=", hardware, " ",
            "simulation_controllers:=", initial_joint_controllers, " ",

        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rbs_arm"), "config/moveit", "rbs_arm.srdf.xacro"]
            ),
            " ",
            "name:=","rbs_arm"," ",
            "prefix:=",prefix," ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("rbs_arm"), "moveit/config", "kinematics.yaml"]
    )

    moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(description_package),
                    'launch',
                    'moveit.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_description': robot_description_content,
                'robot_description_semantic': robot_description_semantic_content,
                'prefix': prefix,
            }.items())


    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
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
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics
        ],

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
            '-name', "rbs_arm",
            '-x', '0.0',
            '-z', '0.0',
            '-y', '0.0',
            '-string', robot_description_content
        ],
        output='screen'
    )

    nodes_to_start = [
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        gazebo_spawn_robot,
        moveit
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
